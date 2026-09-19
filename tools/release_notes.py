"""Print the release notes for one version, taken from CHANGELOG.md.

  python tools/release_notes.py 2.0.0 > notes.md

Exits with an error if the changelog has no section for that version, or an
empty one, so a release cannot go out without notes. An install section with
this version's wheel URL and checksum instructions is appended.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPOSITORY = "vixhvajit/Ster-Vis"


def section(changelog: str, version: str) -> str:
    """The body of '## [version]', up to the next '## ' heading or the link list."""
    pattern = rf"^## \[{re.escape(version)}\][^\n]*\n(.*?)(?=^## |^\[[^\]]+\]: |\Z)"
    match = re.search(pattern, changelog, flags=re.MULTILINE | re.DOTALL)
    if not match:
        raise SystemExit(f"CHANGELOG.md has no section for {version}")
    body = match.group(1).strip()
    if not body:
        raise SystemExit(f"CHANGELOG.md's section for {version} is empty")
    return body


def install_block(version: str) -> str:
    wheel = f"ster_vis-{version}-py3-none-any.whl"
    url = f"https://github.com/{REPOSITORY}/releases/download/v{version}/{wheel}"
    return f"""## Install or upgrade

```bash
ster-vis upgrade                 # from 2.0.0 onwards: verifies the checksum, then installs
pip install {url}
```

On Raspberry Pi OS Bookworm or with ROS 2 Jazzy, add `-c constraints-numpy1.txt`
from the repository to the pip command; `ster-vis upgrade` does this itself.
The files below were built by CI from this tag, and `SHA256SUMS` lists their
checksums. See [docs/RELEASING.md](https://github.com/{REPOSITORY}/blob/v{version}/docs/RELEASING.md)
for what version numbers promise."""


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        print(__doc__)
        return 2
    version = argv[0].lstrip("v")
    changelog = (Path(__file__).resolve().parents[1] / "CHANGELOG.md").read_text(encoding="utf-8")
    print(section(changelog, version) + "\n\n" + install_block(version))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
