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
    generated = {".venv", ".git", ".pytest_cache", "__pycache__", "build", "dist"}
    offenders = []
    for directory, subdirectories, filenames in os.walk(ROOT):
        subdirectories[:] = [name for name in subdirectories if name not in generated]
        for name in filenames:
            path = Path(directory) / name
            relative = path.relative_to(ROOT)
            published_name = relative.as_posix().lower().encode()
            content = path.read_bytes().lower()
            if any(marker in published_name or marker in content for marker in markers):
                offenders.append(relative)
    assert not offenders, f"Private identifiers found in {offenders}"


def test_troubleshooting_covers_every_error():
    guide = (ROOT / "docs" / "troubleshooting.md").read_text(encoding="utf-8")
    error_names = {
        name for name, cls in vars(errors).items() if isinstance(cls, type) and issubclass(cls, errors.Error)
    }
    missing = {name for name in error_names if not re.search(rf"\b{re.escape(name)}\b", guide)}
    assert not missing


PERMANENT_ID = ("The ID you set in Dynamixel Wizard before installing is permanent in v1; changing it after flashing is "
                "not supported (except `openxl330 update --allow-id-change`, which moves a servo back to its stock NV "
                "ID).")


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
    assert "3.5–7.0 V" in readme and "65 °C" in readme and "1.0 A RMS" in readme
    guide = (ROOT / "docs" / "api.md").read_text(encoding="utf-8")
    section = guide.split("## Protections", 1)[1].split("\n## ", 1)[0]
    assert "3.5 V" in section and "7.0 V" in section and "65 °C" in section and "1.0 A" in section
    snippet = re.search(r"(?ms)^```python[ \t]*\n(.*?)^```", section).group(1)
    exec(compile(snippet, "docs/api.md protections", "exec"), {})
    assert capsys.readouterr().out.strip() == "1.0 A RMS, 65 °C, 7.0 V, now 0.0 A RMS"


def test_internal_notes_are_not_published():
    assert not (ROOT / "CLI_NOTES.md").exists()
    assert "CLI_NOTES" not in (ROOT / "pyproject.toml").read_text(encoding="utf-8")


def test_binary_images_are_marked_binary_for_git():
    rules = (ROOT / ".gitattributes").read_text(encoding="utf-8").split("\n")
    assert "*.bin binary" in rules and "*.enc binary" in rules
