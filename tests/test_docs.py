"""Run the documented quickstart and examples on the simulator."""
from __future__ import annotations

import os
import re
import runpy
import subprocess
import sys
from pathlib import Path

import pytest

from openxl330 import errors, images
from openxl330 import bus as bus_module

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ("scan.py", "install_one.py", "arm_six.py", "read_torque.py", "calibrate.py", "first_run.py",
            "gripper.py")


@pytest.fixture
def offline(monkeypatch, tmp_path):
    def open_serial(*args, **kwargs):
        pytest.fail("a simulator example attempted to open a serial port")

    monkeypatch.setattr(bus_module, "_open_serial", open_serial)
    monkeypatch.setenv("OPENXL330_HOME", str(tmp_path / "state"))
    monkeypatch.chdir(tmp_path)


EXAMPLE_ARGS = {"install_one.py": ["--stock-id", "4"]}


@pytest.mark.parametrize("name", EXAMPLES)
def test_examples_run_on_sim(name, offline, monkeypatch):
    example = ROOT / "examples" / name
    assert example.is_file()
    monkeypatch.syspath_prepend(str(example.parent))
    monkeypatch.setattr(sys, "argv", [str(example), "--sim", "6", *EXAMPLE_ARGS.get(name, [])])
    try:
        runpy.run_path(str(example), run_name="__main__")
    except SystemExit as result:
        assert result.code == 0


def test_install_example_needs_the_stock_id(offline, monkeypatch, capsys):
    """Like the CLI, the install example has no default ID."""
    example = ROOT / "examples" / "install_one.py"
    monkeypatch.syspath_prepend(str(example.parent))
    monkeypatch.setattr(sys, "argv", [str(example), "--sim", "6"])
    with pytest.raises(SystemExit) as result:
        runpy.run_path(str(example), run_name="__main__")
    assert result.value.code == 2 and "--stock-id" in capsys.readouterr().err
    text = example.read_text(encoding="utf-8")
    assert "default=1" not in text and "required=True" in text


def test_readme_quickstart_runs_on_sim(offline):
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    match = re.search(r"(?ms)^```python[ \t]*\n(.*?)^```[ \t]*$", readme)
    assert match, "README needs a fenced Python quickstart"
    snippet = match.group(1)
    assert len(snippet.splitlines()) == 10
    connection = 'Bus("/dev/ttyUSB0")'
    assert snippet.count(connection) == 1
    simulated = snippet.replace(connection, "Bus.simulated(3)")
    exec(compile(simulated, "README.md quickstart", "exec"), {})
    assert images.DEFAULT_RELEASE in readme


def test_generated_register_reference_is_current():
    generator = ROOT / "docs" / "generate_registers.py"
    result = subprocess.run([sys.executable, str(generator), "--check"], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_no_private_paths():
    # Split these literals so the checks also scan this file.
    fragments = (
        ("/Use", "rs/"),
        ("XL330-", "Firmware-Flash"),
        ("neuro", "adaptive"),
        ("Unit", "B"),
        ("jor", "dan"),
    )
    markers = ["".join(parts).lower().encode() for parts in fragments]
    offenders = []
    for path in _publishable_files():
        relative = path.relative_to(ROOT)
        published_name = relative.as_posix().lower().encode()
        content = path.read_bytes().lower()
        if any(marker in published_name or marker in content for marker in markers):
            offenders.append(relative)
    assert not offenders, f"Private identifiers found in {offenders}"


def _publishable_files():
    """Every file a commit could publish: in a git checkout, the tracked and the untracked-but-not-ignored files (so a
    worktree's .git file and ignored local folders are not scanned); elsewhere (an unpacked sdist), every file outside
    the generated folders."""
    try:
        listed = subprocess.run(["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"], cwd=ROOT,
                                capture_output=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        listed = None
    if listed:
        return [ROOT / name.decode() for name in listed.split(b"\0") if name and (ROOT / name.decode()).is_file()]
    generated = {".venv", ".git", ".pytest_cache", "__pycache__", "build", "dist"}
    files = []
    for directory, subdirectories, filenames in os.walk(ROOT):
        subdirectories[:] = [name for name in subdirectories if name not in generated]
        files += [Path(directory) / name for name in filenames]
    return files


def test_troubleshooting_covers_every_error():
    guide = (ROOT / "docs" / "troubleshooting.md").read_text(encoding="utf-8")
    error_names = {
        name for name, cls in vars(errors).items() if isinstance(cls, type) and issubclass(cls, errors.Error)
    }
    missing = {name for name in error_names if not re.search(rf"\b{re.escape(name)}\b", guide)}
    assert not missing


PERMANENT_ID = ("Installing keeps the ID the servo has on its stock firmware, so set a unique ID in Dynamixel Wizard "
                "first; afterwards change it over the bus with `openxl330 set-id` (Dynamixel Wizard cannot change the "
                "ID of a servo running openxl330).")


def _text(relative):
    return " ".join((ROOT / relative).read_text(encoding="utf-8").split())


def test_docs_say_the_id_is_permanent():
    for relative in ("README.md", "docs/install.md"):
        assert PERMANENT_ID in _text(relative), relative


def test_docs_state_the_hardware_status_and_windows_latency():
    for relative in ("README.md", "docs/adapters.md"):
        text = _text(relative)
        assert "expected to work, not yet hardware-tested; reports welcome" in text, relative
        assert "16 ms" in text and "1 ms" in text and "Device Manager" in text and "Advanced" in text, relative


def test_readme_installs_from_pypi_first():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    section = readme.split("## Install the Python package", 1)[1].split("\n## ", 1)[0]
    first_block = re.search(r"(?ms)^```sh[ \t]*\n(.*?)^```", section).group(1)
    assert first_block.strip() == "pip install openxl330"


def test_protection_defaults_are_documented_and_the_snippet_runs(offline, capsys):
    readme = _text("README.md")
    assert "3.5–8.0 V" in readme and "65 °C" in readme and "1.0 A RMS" in readme
    guide = (ROOT / "docs" / "api.md").read_text(encoding="utf-8")
    section = guide.split("## Protections", 1)[1].split("\n## ", 1)[0]
    assert "3.5 V" in section and "8.0 V" in section and "65 °C" in section and "1.0 A" in section
    snippet = re.search(r"(?ms)^```python[ \t]*\n(.*?)^```", section).group(1)
    exec(compile(snippet, "docs/api.md protections", "exec"), {})
    assert capsys.readouterr().out.strip() == "1.0 A RMS, 65 °C, 8.0 V, now 0.0 A RMS"


def test_internal_notes_are_not_published():
    assert not (ROOT / "CLI_NOTES.md").exists()
    assert "CLI_NOTES" not in (ROOT / "pyproject.toml").read_text(encoding="utf-8")


def test_binary_images_are_marked_binary_for_git():
    rules = (ROOT / ".gitattributes").read_text(encoding="utf-8").split("\n")
    assert "*.bin binary" in rules and "*.enc binary" in rules


def test_models_doc_lists_every_model():
    from openxl330 import models
    text = _text("docs/models.md")
    for m in models.MODELS.values():
        assert m.name in text and str(m.number) in text, m.name
        assert ("beta" in text.lower()) if m.beta else True
    assert "77.5" in text and "288.4" in text and "--beta" in text


def test_docs_describe_set_id():
    api, install_doc = _text("docs/api.md"), _text("docs/install.md")
    assert "servo.set_id(" in api and "allow_beta" in api
    assert "openxl330 set-id" in install_doc and "--beta" in install_doc
