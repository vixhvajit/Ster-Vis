"""ster-vis upgrade, against a fake GitHub: nothing reaches the network."""

from __future__ import annotations

import hashlib
import io
import json
import urllib.error

import pytest

from stereo_vision import __version__
from stereo_vision.cli import upgrade

WHEEL = b"pretend wheel bytes"


def release(version: str, checksum: str | None = None, with_sums: bool = True) -> dict:
    name = f"ster_vis-{version}-py3-none-any.whl"
    assets = [{"name": name, "browser_download_url": f"https://x/{name}"}]
    if with_sums:
        assets.append({"name": "SHA256SUMS", "browser_download_url": "https://x/SHA256SUMS"})
    return {"tag_name": f"v{version}", "html_url": f"https://x/v{version}", "assets": assets,
            "_sums": f"{checksum or hashlib.sha256(WHEEL).hexdigest()}  {name}\n"}


@pytest.fixture
def github(monkeypatch):
    """Serve fake releases; record pip commands instead of running them."""
    state = {"releases": {}, "latest": None, "pip": []}

    def fake_urlopen(request, timeout=0):
        url = request.full_url
        if url.endswith("/latest"):
            data = state["releases"].get(state["latest"])
        elif "/tags/" in url:
            data = state["releases"].get(url.rsplit("/v", 1)[1])
        elif url.endswith("SHA256SUMS"):
            return io.BytesIO(next(iter(state["releases"].values()))["_sums"].encode())
        else:
            return io.BytesIO(WHEEL)
        if data is None:
            raise urllib.error.HTTPError(url, 404, "Not Found", {}, None)
        return io.BytesIO(json.dumps(data).encode())

    class Done:
        returncode = 0

    monkeypatch.setattr(upgrade.urllib.request, "urlopen", fake_urlopen)
    monkeypatch.setattr(upgrade.subprocess, "run", lambda command: state["pip"].append(command) or Done())
    monkeypatch.setattr(upgrade, "needs_numpy1", lambda: False)
    monkeypatch.setattr(upgrade, "launched_from_windows_exe", lambda: False)
    return state


def bump(version: str, part: int) -> str:
    numbers = [int(x) for x in version.split(".")[:3]]
    numbers[part] += 1
    return ".".join(map(str, numbers[: part + 1] + [0] * (2 - part)))


def test_version_ordering():
    assert upgrade.version_key("2.0.0") > upgrade.version_key("1.9.9")
    assert upgrade.version_key("2.0.0") > upgrade.version_key("2.0.0-rc.1")
    assert upgrade.version_key("v2.1.0") > upgrade.version_key("2.0.10")
    with pytest.raises(ValueError):
        upgrade.version_key("latest")


def test_check_reports_a_newer_release(github, capsys):
    newer = bump(__version__, 1)
    github["releases"][newer] = release(newer)
    github["latest"] = newer
    assert upgrade.main(["--check"]) == 0
    assert f"{newer} is available" in capsys.readouterr().out
    assert github["pip"] == []


def test_upgrades_after_verifying_the_checksum(github, capsys):
    newer = bump(__version__, 2)
    github["releases"][newer] = release(newer)
    github["latest"] = newer
    assert upgrade.main([]) == 0
    out = capsys.readouterr().out
    assert "checksum verified" in out
    (command,) = github["pip"]
    assert command[1:4] == ["-m", "pip", "install"] and command[4].endswith(f"ster_vis-{newer}-py3-none-any.whl")


def test_refuses_a_wheel_whose_checksum_does_not_match(github, capsys):
    newer = bump(__version__, 2)
    github["releases"][newer] = release(newer, checksum="0" * 64)
    github["latest"] = newer
    assert upgrade.main([]) == 1
    assert "checksum mismatch" in capsys.readouterr().out
    assert github["pip"] == []


def test_refuses_a_release_without_checksums(github, capsys):
    newer = bump(__version__, 2)
    github["releases"][newer] = release(newer, with_sums=False)
    github["latest"] = newer
    assert upgrade.main([]) == 1
    assert github["pip"] == []


def test_up_to_date_installs_nothing(github, capsys):
    github["releases"][__version__] = release(__version__)
    github["latest"] = __version__
    assert upgrade.main([]) == 0
    assert "already up to date" in capsys.readouterr().out
    assert github["pip"] == []


def test_rolls_back_to_an_older_version(github):
    github["releases"]["0.1.0"] = release("0.1.0")
    assert upgrade.main(["--to", "0.1.0"]) == 0
    (command,) = github["pip"]
    assert command[4].endswith("ster_vis-0.1.0-py3-none-any.whl")
    assert "--force-reinstall" not in command  # a different version replaces the installed one


def test_reinstalling_the_same_version_needs_force(github, capsys):
    github["releases"][__version__] = release(__version__)
    assert upgrade.main(["--to", __version__]) == 0
    assert github["pip"] == []
    assert upgrade.main(["--to", __version__, "--force"]) == 0
    assert "--force-reinstall" in github["pip"][0] and "--no-deps" in github["pip"][0]


def test_keeps_numpy_1_on_systems_that_need_it(github, monkeypatch):
    monkeypatch.setattr(upgrade, "needs_numpy1", lambda: True)
    newer = bump(__version__, 2)
    github["releases"][newer] = release(newer)
    github["latest"] = newer
    assert upgrade.main([]) == 0
    command = github["pip"][0]
    assert "-c" in command and command[command.index("-c") + 1].endswith("constraints-numpy1.txt")


def test_pins_match_the_constraints_file():
    from pathlib import Path

    pins = [line for line in Path("constraints-numpy1.txt").read_text().splitlines() if line and not line.startswith("#")]
    assert pins == upgrade.NUMPY1_PINS.strip().splitlines()


def test_unknown_version_is_a_clear_error(github, capsys):
    assert upgrade.main(["--to", "9.9.9"]) == 1
    assert "no release 9.9.9" in capsys.readouterr().out


def test_windows_exe_points_to_python_m_instead_of_breaking_itself(github, monkeypatch, capsys):
    """Found by a real rollback: Windows kills ster-vis.exe when pip replaces it."""
    monkeypatch.setattr(upgrade, "launched_from_windows_exe", lambda: True)
    newer = bump(__version__, 2)
    github["releases"][newer] = release(newer)
    github["latest"] = newer
    assert upgrade.main(["--to", newer]) == 1
    out = capsys.readouterr().out
    assert "-m stereo_vision upgrade --to" in out
    assert github["pip"] == []  # stopped before downloading or installing


def test_check_works_from_the_windows_exe(github, monkeypatch, capsys):
    monkeypatch.setattr(upgrade, "launched_from_windows_exe", lambda: True)
    github["releases"][__version__] = release(__version__)
    github["latest"] = __version__
    assert upgrade.main(["--check"]) == 0


def test_a_failed_pip_does_not_claim_the_old_version_survived(github, monkeypatch, capsys):
    class Failed:
        returncode = 1

    monkeypatch.setattr(upgrade.subprocess, "run", lambda command: Failed())
    newer = bump(__version__, 2)
    github["releases"][newer] = release(newer)
    github["latest"] = newer
    assert upgrade.main([]) == 1
    out = capsys.readouterr().out
    assert "still installed" not in out and "ster-vis --version" in out
