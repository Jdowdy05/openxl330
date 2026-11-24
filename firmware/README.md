# openxl330 firmware source (FW231 fast17)

This folder is the source of the firmware the Python package bundles: release 17, register map ABI 5. Its source ID is `0x77e22d48`, the first four bytes of the source tree hash in [SOURCES.md](SOURCES.md). It is open-source under the MIT licence ([LICENSE](LICENSE)). `firmware/bam_native.c` adapts code from BAM under Apache-2.0: see [NOTICE](NOTICE), [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and [LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt).

> **Not yet run on hardware.** fast17's new features are verified in the native tests and in simulation only: the ID change over the bus, sensor reacquire, and the voltage warning and switch. The XL330-M077 images are a beta, with constants estimated from the gear ratio ([docs/M077_BETA.md](docs/M077_BETA.md)).

## Build

You need `arm-none-eabi-gcc` 15.3.1 (Arm GNU Toolchain 15.3.Rel1) and Python 3. From this folder:

| Image | Command | Output |
| --- | --- | --- |
| `fw231-comp2-fast17` (XL330-M288, performance) | `COMP=1 FW_SOURCE_ID=0x77e22d48 sh build.sh` | `build/role-1-comp/candidate.bin` |
| `fw231-comp2-fast17-log` (XL330-M288, logging) | `COMP=1 LOG=1 IDLOG=0 FW_SOURCE_ID=0x77e22d48 sh build.sh` | `build/role-1-comp-log/candidate.bin` |
| `fw231-comp2-fast17-m077` (XL330-M077, performance) | `COMP=1 GEAR=77 FW_SOURCE_ID=0x77e22d48 sh build.sh` | `build/role-1-comp-m077/candidate.bin` |
| `fw231-comp2-fast17-m077-log` (XL330-M077, logging) | `COMP=1 GEAR=77 LOG=1 IDLOG=0 FW_SOURCE_ID=0x77e22d48 sh build.sh` | `build/role-1-comp-m077-log/candidate.bin` |

With that toolchain, each `candidate.bin` equals the bundled `src/openxl330/images/<image>/firmware.bin` byte for byte. Three things tie the binary to this source:
- its manifest's `source_tree_sha256` starts with `77e22d48`;
- the release ID the servo reports (read 1120, word 1) is `0x77e22d48`;
- `FW_SOURCE_ID` is the value written into the image.

`build.sh` checks each image with `tests/check_image.py`. A performance image must end below the ID-log page at 0x0800D800.

## Native tests

`sh test_native.sh` builds and runs the host-side tests with clang, AddressSanitizer and UBSan. It covers:
- the controller;
- the bus, the ID log and sensor reacquire;
- the voltage protection;
- golden control traces for every image variant.

## How the bundled image relates to this source

openxl330 installs through the servo's retained ROBOTIS loader. The loader takes the program in its own upload encoding: `firmware.enc` beside each bundled `firmware.bin`. The package ships both. The encoding step is not part of this repository, so openxl330 installs only the bundled, pinned images, and an image you build yourself cannot be installed with openxl330 as it stands. Each bundled image's manifest pins both files by SHA-256.

## Files

[SOURCES.md](SOURCES.md) lists every file of the release source with its SHA-256. This README and `LICENSES/Apache-2.0.txt` were added when the tree was published.
