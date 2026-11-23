"""Reject malformed local package metadata before preparing an upload."""
from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

from openxl330 import Error, images


@pytest.mark.parametrize("updates, message", [
    ({"raw_file": "other.bin"}, "firmware.bin"),
    ({"encoded_file": "other.enc"}, "firmware.enc"),
    ({"version": 52}, "firmware version"),
    ({"marker": "other marker"}, "marker"),
    ({"known_loaders": {"0x123456": "invalid CRC"}}, "known_loaders"),
    ({"known_loaders": {"abcdef12": ""}}, "known_loaders"),
])
def test_manifest_rejects_wrong_release_metadata(tmp_path, updates, message):
    name = images.DEFAULT_RELEASE
    source = Path(images.__file__).with_name("images") / name
    folder = tmp_path / name
    shutil.copytree(source, folder)
    shutil.copyfile(folder / "firmware.bin", folder / "other.bin")
    shutil.copyfile(folder / "firmware.enc", folder / "other.enc")
    manifest = folder / "manifest.json"
    metadata = json.loads(manifest.read_text(encoding="utf-8"))
    metadata.update(updates)
    manifest.write_text(json.dumps(metadata), encoding="utf-8")
    with pytest.raises(Error, match=message):
        images.Release(name, root=tmp_path)


def test_manifest_requires_full_source_digest(tmp_path):
    name = images.DEFAULT_RELEASE
    source = Path(images.__file__).with_name("images") / name
    folder = tmp_path / name
    shutil.copytree(source, folder)
    manifest = folder / "manifest.json"
    metadata = json.loads(manifest.read_text(encoding="utf-8"))
    metadata["source_tree_sha256"] = metadata["source_tree_sha256"][:8] + "g" * 56
    manifest.write_text(json.dumps(metadata), encoding="utf-8")
    with pytest.raises(Error, match="source hash"):
        images.Release(name, root=tmp_path)
