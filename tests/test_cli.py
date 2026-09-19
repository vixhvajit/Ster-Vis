"""The ster-vis command and packaging: every command answers, nothing reads repo paths."""

from __future__ import annotations

import urllib.request

import pytest

from stereo_vision import __version__
from stereo_vision.cli import main as cli
from stereo_vision.cli.viewer import make_server


def test_version(capsys):
    assert cli.main(["--version"]) == 0
    assert __version__ in capsys.readouterr().out


def test_help_lists_every_command(capsys):
    assert cli.main([]) == 0
    out = capsys.readouterr().out
    for name in cli.COMMANDS:
        assert name in out


def test_unknown_command_fails_with_usage(capsys):
    assert cli.main(["teleport"]) == 2
    assert "unknown command" in capsys.readouterr().err


@pytest.mark.parametrize("command", list(cli.COMMANDS))
def test_every_command_has_help(command, capsys):
    with pytest.raises(SystemExit) as exit_:
        cli.main([command, "--help"])
    assert exit_.value.code == 0
    assert f"ster-vis {command}" in capsys.readouterr().out


def test_chessboard_writes_where_it_is_run(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert cli.main(["chessboard"]) == 0
    assert (tmp_path / "docs" / "chessboard_9x6_25mm_a4.pdf").is_file()


def test_doctor_runs_and_reports(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    code = cli.main(["doctor"])
    out = capsys.readouterr().out
    assert "OpenCV" in out and "calibration" in out
    assert code == 0  # a missing calibration is a warning, not a failure


class TestViewerServer:
    def fetch(self, server, path):
        with urllib.request.urlopen(f"http://127.0.0.1:{server.server_address[1]}{path}", timeout=5) as r:
            return r.status, r.read()

    def run(self, file=None):
        import threading

        server = make_server(file)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        return server

    def test_serves_the_packaged_page_and_the_named_file(self, tmp_path):
        cloud = tmp_path / "cloud.ply"
        cloud.write_bytes(b"ply\nformat ascii 1.0\nelement vertex 0\nend_header\n")
        server = self.run(cloud)
        try:
            status, page = self.fetch(server, "/")
            assert status == 200 and b"Ster-Vis viewer" in page
            status, body = self.fetch(server, "/file/cloud.ply")
            assert body.startswith(b"ply")
        finally:
            server.shutdown()
            server.server_close()

    def test_refuses_paths_outside_the_samples_folder(self, tmp_path, monkeypatch):
        monkeypatch.chdir(tmp_path)
        (tmp_path / "secret.txt").write_text("private")
        (tmp_path / "docs" / "samples").mkdir(parents=True)
        server = self.run()
        try:
            for path in ("/samples/../../secret.txt", "/samples/%2e%2e/%2e%2e/secret.txt", "/secret.txt"):
                with pytest.raises(urllib.error.HTTPError) as error:
                    self.fetch(server, path)
                assert error.value.code == 404
        finally:
            server.shutdown()
            server.server_close()


class TestReleaseTooling:
    """The release workflow depends on these; keep them honest."""

    def notes(self):
        import importlib.util
        from pathlib import Path

        spec = importlib.util.spec_from_file_location("release_notes", Path("tools/release_notes.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module, Path("CHANGELOG.md").read_text(encoding="utf-8")

    def test_changelog_has_notes_for_the_current_version(self):
        module, changelog = self.notes()
        body = module.section(changelog, __version__)
        assert body.strip()

    def test_a_section_stops_at_the_next_version_and_before_the_links(self):
        module, changelog = self.notes()
        oldest = module.section(changelog, "0.1.0")
        assert "## [" not in oldest and "]: https://" not in oldest

    def test_missing_versions_stop_the_release(self):
        module, changelog = self.notes()
        with pytest.raises(SystemExit):
            module.section(changelog, "99.0.0")

    def test_changelog_links_the_current_version(self):
        _, changelog = self.notes()
        assert f"[{__version__}]: https://github.com/vixhvajit/Ster-Vis/" in changelog
