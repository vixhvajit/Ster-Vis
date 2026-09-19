# Versions, releases, patches and upgrades

## What a version number promises

Ster-Vis follows [Semantic Versioning](https://semver.org/): `MAJOR.MINOR.PATCH`.

| Bump | When | What you need to do |
|---|---|---|
| **Patch** `2.0.x` | bug fixes only; nothing you use changes | upgrade freely |
| **Minor** `2.x.0` | new features, new options, new endpoints; everything existing keeps working | upgrade freely |
| **Major** `x.0.0` | something you may rely on changes or goes away | read the *Breaking* section of the [changelog](../CHANGELOG.md) first |

These are the interfaces the promise covers. Changing any of them in an
incompatible way needs a major version:

- **The `ster-vis` command:** command names, option names and their meaning,
  and exit codes.
- **The HTTP API:** paths, field names, units and meanings under `/api/v1/`.
  Incompatible changes go to a new `/api/v2/`, and `/api/v1/` keeps working
  alongside it for at least one further major version.
- **ROS 2:** topic names, message types, frame names and units.
- **Files:** calibration `.npz` files, depth PNGs and `.npy` files, and
  recordings. A newer version always reads files written by an older one.
- **The Python API:** the names exported from `stereo_vision` (its
  `__all__`). Other modules are internal and can change in any release.
- **Install files:** `constraints-numpy1.txt` and the `deploy/pi/` scripts
  and paths.

Adding a field to a JSON response, a topic, or an option is a minor change, so
clients should ignore fields they don't know.

Output *values* can improve in any release, since better accuracy is the
point. If a change could make a robot behave differently, for example the
laser scan filling gaps differently, the changelog says so.

## Which versions get fixes

- **The latest release gets every bug fix**, as a patch release.
- **The previous major version** gets fixes for security problems and serious
  bugs (crashes, wrong measurements) for six months after the next major
  comes out. After 2.0.0 that means 0.1.x, until 2027-03-19.
- Older versions get no fixes; upgrade instead.

## How to upgrade

Check for a newer release, then install it:

```bash
ster-vis upgrade --check
ster-vis upgrade
```

`ster-vis upgrade` downloads the wheel from the GitHub release and **checks it
against the release's `SHA256SUMS`** before installing. On systems whose
packages need numpy 1.x (Raspberry Pi OS Bookworm, ROS 2 Jazzy), it applies
the same pins as `constraints-numpy1.txt` automatically. To go back to an
earlier version:

```bash
ster-vis upgrade --to 0.1.0
```

On Windows, run it as `python -m stereo_vision upgrade` instead. The
`ster-vis.exe` launcher can't replace itself while it runs; from 2.0.1 it says
so rather than trying.

On a Pi installed with `deploy/pi/install.sh`, run it with `sudo` and restart
the service:

```bash
sudo ster-vis upgrade
sudo systemctl restart ster-vis
```

Or update the checkout and re-run the installer, which keeps your settings:

```bash
git fetch --tags && git checkout v2.0.1      # the version you want
sudo deploy/pi/install.sh
```

Calibrations carry over: every version reads files from older versions.
Recalibrate only if the changelog says so.

`ster-vis` 0.1.0 has no `upgrade` command. Upgrade from it with pip once:

```bash
pip install https://github.com/vixhvajit/Ster-Vis/releases/download/v2.0.1/ster_vis-2.0.1-py3-none-any.whl
```

## Reporting a bug

Open an issue with the *Bug report* template. It asks for the output of
`ster-vis doctor`, which covers the versions, board, cameras and calibration
state that most bugs depend on.

## Making a release (maintainers)

Releases are built and published by
[`.github/workflows/release.yml`](../.github/workflows/release.yml) when a
`v*` tag is pushed. It refuses to publish if the tag doesn't match
`__version__`, or if the changelog has no section for that version.

1. Move the entries under `## [Unreleased]` in `CHANGELOG.md` into a new
   `## [X.Y.Z] - YYYY-MM-DD` section, and update the comparison links at the
   bottom.
2. Set `__version__` in `src/stereo_vision/__init__.py` to `X.Y.Z`.
3. Commit (`release: X.Y.Z`), push, and wait for CI to pass.
4. Optionally, do a dry run: run the release workflow by hand (Actions →
   Release → Run workflow, or `gh workflow run release.yml`). Run by hand, it
   tests, builds and checks everything but doesn't publish.
5. Tag and push:

   ```bash
   git tag -a vX.Y.Z -m "Ster-Vis X.Y.Z"
   git push origin vX.Y.Z
   ```

The workflow runs the test suite, builds the wheel and source archive, writes
`SHA256SUMS`, and publishes a GitHub release. The notes come from the
changelog section, and the files are attached.

## Making a patch release

Fixes land on `main` first, with a test that fails without the fix. Then:

- **If `main` has nothing unreleased but fixes**, release a patch straight
  from `main` as above.
- **If `main` already holds features for the next minor**, patch from a
  release branch instead:

  ```bash
  git switch -c release/2.0 v2.0.0     # once per minor; reuse it afterwards
  git cherry-pick <fix commit>
  # update CHANGELOG.md and __version__ to 2.0.1 on this branch
  git push -u origin release/2.0
  git tag -a v2.0.1 -m "Ster-Vis 2.0.1" && git push origin v2.0.1
  ```

  Then copy the `2.0.1` changelog section back to `main`, so the history
  stays complete.

The same applies to the previous major during its support window, on a
`release/0.1`-style branch.
