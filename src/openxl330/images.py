"""The bundled FW231 release: one identical, pre-encoded upload for every servo. No I/O to devices, and no codec.

Each release folder holds
  firmware.bin   our own firmware's program bytes (plaintext), with the default device block: a servo running it
                 answers at the ID it had on its stock firmware (its NV journal ID) and starts at 1 Mbaud;
  firmware.enc   the same program, padded (pad_program) and encoded for the retained ROBOTIS loader: the upload,
                 byte for byte as it is sent (encoded when the release was built; this package cannot encode or
                 decode);
  manifest.json  sizes and SHA-256 pins of both, the source-tree hash, the release's variant, its release number and
                 ABI (what read 1120 reports) and the loaders it is known to install over (known_loaders).
Release(name) verifies all of them, including that the program carries the release-ID words read 1120 reports. The
program region is 0x08004000..0x0800DFFF (40960 B). The loader (0x08000000..0x08003FFF) and each servo's own
calibration/NV journal (0x0800E000..0x0800FFFF) are never part of an image.

There is no per-servo patching and no encoding: the upload is the shipped firmware.enc. plaintext_for() maps a bundled
upload back to its program bytes (a lookup for the simulator, not a decoder); release_id is what read 1120 must report;
known_loaders() is the installer's list of known loaders.
"""
from __future__ import annotations

import functools
import hashlib
import json
import re
from dataclasses import dataclass
from importlib import resources
from pathlib import Path

from ._protocol import framing, packets
from . import models
from .errors import Error, InvalidArgument
from .registers import (DEVCFG_BYTES, DEVCFG_OFFSET, FW231_MARKER, RELEASE_VARIANTS,
                        encode_release_id,
                        parse_devcfg)

PROGRAM_BASE = 0x08004000
PROGRAM_LIMIT = 0x0800E000          # first byte of the servo's own calibration/NVM
PROGRAM_MAX = PROGRAM_LIMIT - PROGRAM_BASE

# Newest first. The first entry is the default. Bundle only qualified firmware.
# 0.2.0: fast17 (release 17, ABI 5) for the XL330-M288 and, as a beta, the XL330-M077, each as a performance image (the
# default) and a logging image (the DAQ drain and extra counters; no ID log). release_for() picks one.
RELEASES = ("fw231-comp2-fast17", "fw231-comp2-fast17-log", "fw231-comp2-fast17-m077", "fw231-comp2-fast17-m077-log")
DEFAULT_RELEASE = RELEASES[0]
VARIANTS = ("performance", "logging")

_CORRUPT = "bundled image {name} is corrupt ({why}); reinstall openxl330"


def release_names() -> list:
    """Names of the bundled releases, newest first."""
    return list(RELEASES)


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def pad_program(raw: bytes) -> bytes:
    """FF-pad to a multiple of 8, plus 8 more FF if the result is a multiple of 128 (the loader's rule)."""
    raw = raw + b"\xff" * ((-len(raw)) % 8)
    if len(raw) % 128 == 0:
        raw += b"\xff" * 8
    return raw


@dataclass(frozen=True)
class Packaged:
    raw: bytes            # padded program bytes as they will sit at 0x08004000
    encoded: bytes        # the upload: the release's shipped firmware.enc
    packets: tuple        # F7 requests
    progress: tuple       # expected (index, bytes) per F7 reply

    @property
    def devcfg(self) -> bytes:
        """The 32-byte device block inside raw (the default one in every bundled release)."""
        return self.raw[DEVCFG_OFFSET:DEVCFG_OFFSET + DEVCFG_BYTES]

    @functools.cached_property
    def raw_sha256(self) -> str:
        return _sha256(self.raw)

    @functools.cached_property
    def encoded_sha256(self) -> str:
        return _sha256(self.encoded)

    @functools.cached_property
    def crc16(self) -> int:
        return framing.crc16(self.encoded)


class Release:
    """A bundled release: manifest.json, the program (firmware.bin) and its upload (firmware.enc), verified against the
    manifest pins. `root` (a folder holding <name>/) is for tests; the default is the installed package.

    Attributes from the verified manifest: name, version, marker; release_id = what read 1120 reports on a servo
    running this release, dict(number, variant, abi, source_id) with source_id the first 4 bytes of
    source_tree_sha256 (the program carries these words); known_loaders = {CRC-32 of the retained loader: name}, the
    loaders this release is known to install over."""

    def __init__(self, name: str, root=None):
        if not isinstance(name, str) or name not in RELEASES:
            raise InvalidArgument(f"unknown release {name!r}: choose one of {', '.join(RELEASES)}")
        folder = resources.files("openxl330") / "images" / name if root is None else Path(root) / name
        try:
            m = self.manifest = json.loads((folder / "manifest.json").read_bytes())
            if m["name"] != name:
                raise Error(_CORRUPT.format(name=name, why=f"its manifest names {m['name']!r}"))
            if (m["raw_file"], m["encoded_file"]) != ("firmware.bin", "firmware.enc"):
                raise Error(_CORRUPT.format(name=name, why="manifest must name firmware.bin and firmware.enc"))
            if type(m["version"]) is not int or m["version"] != 231:
                raise Error(_CORRUPT.format(name=name, why="manifest firmware version must be 231"))
            if m["marker"] != FW231_MARKER.decode("ascii"):
                raise Error(_CORRUPT.format(name=name, why="manifest names the wrong firmware marker"))
            self.raw = (folder / "firmware.bin").read_bytes()
            self.encoded = (folder / "firmware.enc").read_bytes()
            if _sha256(self.raw) != m["raw_sha256"] or len(self.raw) != m["raw_bytes"]:
                raise Error(_CORRUPT.format(name=name, why="program does not match its manifest"))
            if FW231_MARKER not in self.raw:
                raise Error(_CORRUPT.format(name=name, why="program does not carry the firmware marker"))
            if _sha256(self.encoded) != m["encoded_sha256"] or len(self.encoded) != m["encoded_bytes"]:
                raise Error(_CORRUPT.format(name=name, why="upload does not match its manifest"))
            padded = pad_program(self.raw)
            if (len(padded) != len(self.encoded) or len(padded) != m["padded_program_bytes"]
                    or _sha256(padded) != m["padded_program_sha256"] or len(padded) > PROGRAM_MAX):
                raise Error(_CORRUPT.format(name=name, why="program and upload sizes disagree"))
            if parse_devcfg(self.raw[DEVCFG_OFFSET:DEVCFG_OFFSET + DEVCFG_BYTES])["state"] != "default":
                raise Error(_CORRUPT.format(name=name, why="no default device block"))
            if (m["variant"] not in RELEASE_VARIANTS
                    or not re.fullmatch(r"[0-9a-fA-F]{64}", m["source_tree_sha256"])):
                raise Error(_CORRUPT.format(name=name, why="unknown variant or source hash"))
            if not (type(m["release_number"]) is int and 0 <= m["release_number"] <= 0xFFFF
                    and type(m["abi"]) is int and 0 <= m["abi"] <= 0xFF):
                raise Error(_CORRUPT.format(name=name, why="no valid release number or ABI"))
            self.release_id = dict(number=m["release_number"], variant=m["variant"], abi=m["abi"],
                                   source_id=int(m["source_tree_sha256"][:8], 16))
            if encode_release_id(**self.release_id)[:8] not in self.raw:
                raise Error(_CORRUPT.format(name=name, why="the program does not carry its manifest's release ID"))
            self.known_loaders = {int(crc, 16): what for crc, what in m["known_loaders"].items()
                                  if re.fullmatch(r"[0-9a-fA-F]{8}", crc) and isinstance(what, str) and what.strip()}
            if not self.known_loaders or len(self.known_loaders) != len(m["known_loaders"]):
                raise Error(_CORRUPT.format(name=name, why="known_loaders is not a table of 8-digit CRC-32 values"))
            self.model = m.get("model", 1200)
            self.set_id = m.get("set_id", False)            # the image has the ID log (register 1144)
            self.kind = m.get("kind", "performance")        # 'performance' or 'logging' (DAQ drain + counters)
            self.beta = m.get("beta", False)
            if (type(self.set_id) is not bool or type(self.beta) is not bool
                    or self.kind not in ("performance", "logging")):
                raise Error(_CORRUPT.format(name=name, why="set_id, beta or kind is not valid"))
            if type(self.model) is not int or models.by_number(self.model) is None or (
                    models.for_variant(m["variant"]) is not models.by_number(self.model)):
                raise Error(_CORRUPT.format(name=name, why="unknown model, or a model its variant is not built for"))
            self.name = name
            self.version = m["version"]
            self.marker = m["marker"]
        except Error:
            raise
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            raise Error(_CORRUPT.format(name=name, why=f"{type(exc).__name__}: {exc}")) from exc
        self._packaged = None

    def packaged(self) -> Packaged:
        """The upload, built from the shipped encoded bytes: its F7 requests, the expected F7 progress, and the padded
        program it puts at 0x08004000. The same for every servo."""
        if self._packaged is None:
            encoded = self.encoded
            requests = tuple(packets.build_f7_upload_packets(encoded))
            progress = ((0, 0),) + tuple((i + 1, min((i + 1) * 128, len(encoded)))
                                         for i in range((len(encoded) + 127) // 128))
            if len(requests) != len(progress):
                raise Error("internal error: upload packet count does not match the progress table; reinstall "
                            "openxl330")
            self._packaged = Packaged(pad_program(self.raw), encoded, requests, progress)
        return self._packaged


@functools.lru_cache(maxsize=None)
def bundled(name: str) -> Release:
    """The bundled release `name`, verified once per process (Release(name) verifies it each time)."""
    return Release(name)


def release_for(model: int, variant: str = "performance") -> str:
    """The bundled release for a servo model (number, e.g. 1200) and variant ('performance', the default, or
    'logging'). InvalidArgument when there is none."""
    if variant not in VARIANTS:
        raise InvalidArgument(f"variant {variant!r}: use one of {', '.join(VARIANTS)}")
    for name in RELEASES:
        release = bundled(name)
        if release.model == model and release.kind == variant:
            return name
    raise InvalidArgument(f"no bundled {variant} image for model {model}")


def known_loaders() -> dict:
    """{CRC-32 (zlib) of a retained loader: its name}: the loaders the bundled releases' manifests list as known (the
    installer's PROVEN_LOADERS)."""
    out = {}
    for name in RELEASES:
        out.update(bundled(name).known_loaders)
    return out


def plaintext_for(encoded: bytes) -> bytes | None:
    """The program bytes (padded, as they sit at 0x08004000) behind a bundled release's upload, found by the SHA-256 of
    `encoded`; None for any other bytes. This is how the simulator's loader knows what an upload installs: the package
    has no decoder."""
    digest = _sha256(bytes(encoded))
    for name in RELEASES:
        release = bundled(name)
        if release.manifest["encoded_sha256"] == digest:
            return release.packaged().raw
    return None
