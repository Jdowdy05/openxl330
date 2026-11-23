"""Per-servo calibration: encoder zero and direction, and counts <-> radians conversion.

rad = direction * (counts - zero_counts) * 2*pi / 4096. Counts are continuous (multi-turn), so no wrapping happens.
The file is $OPENXL330_HOME/calibration.json: {"version": 1, "joints": {"<id>": {"zero_counts": N, "direction": 1}}}.
"""
from __future__ import annotations

import json
import math
import numbers
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Dict, Optional, Union

from . import paths, registers
from .errors import CalibrationError, InvalidArgument

_VERSION = 1
_HINT = ("check the file and its permissions, or move the file aside to use default joint calibration. "
         "With a valid file, `openxl330 calibrate --id ID --reset` resets just that joint")


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _check_servo_id(servo_id: object) -> int:
    if not _is_int(servo_id) or not 0 <= servo_id <= 252:  # type: ignore[operator]
        raise InvalidArgument("servo id must be an int in 0..252, got %r" % (servo_id,))
    return servo_id  # type: ignore[return-value]


@dataclass
class Joint:
    zero_counts: int = registers.CENTER_COUNTS
    direction: int = 1

    def __post_init__(self) -> None:
        if not _is_int(self.zero_counts):
            raise InvalidArgument("zero_counts must be an int, got %r" % (self.zero_counts,))
        if not _is_int(self.direction) or self.direction not in (1, -1):
            raise InvalidArgument("direction must be 1 or -1, got %r" % (self.direction,))


_DEFAULT_JOINT = Joint()


class Calibration:
    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = Path(path) if path is not None else None
        self._joints: Dict[int, Joint] = {}
        self._lock = Lock()

    @classmethod
    def load(cls, path: Union[Path, str, None] = None) -> "Calibration":
        target = Path(path) if path is not None else paths.home() / "calibration.json"
        cal = cls(target)
        try:
            text = target.read_text(encoding="utf-8")
        except FileNotFoundError:
            return cal
        except (OSError, UnicodeError) as exc:
            raise CalibrationError("calibration file %s is unusable (%s); %s" % (target, exc, _HINT)) from exc
        try:
            data = json.loads(text)
            if not isinstance(data, dict) or data.get("version") != _VERSION or not _is_int(data["version"]):
                raise ValueError("unsupported or missing version")
            joints = data.get("joints")
            if not isinstance(joints, dict):
                raise ValueError("'joints' must be an object")
            for key, entry in joints.items():
                servo_id = int(key)
                if str(servo_id) != key:
                    raise ValueError("bad servo id %r" % (key,))
                _check_servo_id(servo_id)
                if not isinstance(entry, dict):
                    raise ValueError("joint %s must be an object" % key)
                cal._joints[servo_id] = Joint(entry["zero_counts"], entry["direction"])
        except (ValueError, KeyError, TypeError) as exc:
            raise CalibrationError("calibration file %s is unusable (%s); %s" % (target, exc, _HINT)) from exc
        return cal

    def save(self) -> None:
        with self._lock:
            target = self.path if self.path is not None else paths.home() / "calibration.json"
            self._save(target, self._joints)
            self.path = target

    @staticmethod
    def _save(target, joints):
        data = {"version": _VERSION,
                "joints": {str(i): {"zero_counts": j.zero_counts, "direction": j.direction}
                           for i, j in sorted(joints.items())}}
        tmp = None
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=target.parent,
                                             prefix=target.name + ".", suffix=".tmp", delete=False) as f:
                tmp = Path(f.name)
                json.dump(data, f, indent=2)
                f.write("\n")
            os.replace(tmp, target)
        except (OSError, ValueError) as exc:
            raise CalibrationError(f"could not save calibration to {target} ({exc}). "
                                   "Check the path and permissions") from exc
        finally:
            if tmp is not None:
                try:
                    tmp.unlink(missing_ok=True)
                except OSError:
                    pass

    def _joint(self, servo_id: int) -> Joint:
        return self._joints.get(_check_servo_id(servo_id), _DEFAULT_JOINT)

    def joint(self, servo_id: int) -> Joint:
        j = self._joint(servo_id)
        return Joint(j.zero_counts, j.direction)

    def set(self, servo_id: int, zero_counts: Optional[int] = None, direction: Optional[int] = None) -> None:
        with self._lock:
            self._joints[servo_id] = self._updated_joint(servo_id, zero_counts, direction)

    def _updated_joint(self, servo_id, zero_counts, direction):
        current = self._joint(servo_id)
        return Joint(current.zero_counts if zero_counts is None else zero_counts,
                     current.direction if direction is None else direction)

    def _apply(self, updates):
        """Install servo/group changes together, once any persistent save succeeds."""
        with self._lock:
            joints = self._joints.copy()
            for servo_id, (zero_counts, direction) in updates.items():
                joints[servo_id] = self._updated_joint(servo_id, zero_counts, direction)
            if self.path is not None:
                self._save(self.path, joints)
            self._joints = joints

    def reset(self, servo_id: Optional[int] = None) -> None:
        with self._lock:
            if servo_id is None:
                self._joints.clear()
            else:
                self._joints.pop(_check_servo_id(servo_id), None)

    def sign(self, servo_id: int) -> int:
        return self._joint(servo_id).direction

    def to_rad(self, servo_id: int, counts: int) -> float:
        j = self._joint(servo_id)
        return j.direction * (counts - j.zero_counts) * 2.0 * math.pi / registers.COUNTS_PER_REV

    def to_counts(self, servo_id: int, rad: float) -> int:
        j = self._joint(servo_id)
        if isinstance(rad, bool) or not isinstance(rad, numbers.Real):
            raise InvalidArgument(f"position must be a real number in radians, got {rad!r}")
        try:
            return j.zero_counts + int(round(j.direction * rad * registers.COUNTS_PER_REV / (2.0 * math.pi)))
        except (ValueError, OverflowError):
            raise InvalidArgument("position in radians cannot be represented in encoder counts") from None
