"""Per-user data root: $OPENXL330_HOME, default ~/.openxl330 (records/, calibration.json, logs/)."""
from __future__ import annotations

import os
from pathlib import Path


def home() -> Path:
    """$OPENXL330_HOME if set (and not empty), else ~/.openxl330. Nothing is created here."""
    value = os.environ.get('OPENXL330_HOME')
    return Path(value).expanduser() if value else Path.home() / '.openxl330'
