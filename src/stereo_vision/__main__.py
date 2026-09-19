"""Allow ``python -m stereo_vision`` as an alternative to ``ster-vis``."""

from stereo_vision.cli.main import main

raise SystemExit(main())
