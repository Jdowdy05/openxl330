# Contributing

Thanks for helping. Bug reports, hardware reports and pull requests are all welcome. This project drives real motors and rewrites servo flash, so a few rules matter more than usual.

## Reporting

- **Bugs and hardware reports:** open an issue with the template. Say which adapter, operating system, Python version, openxl330 version (`openxl330 version`) and firmware release (`openxl330 status --port PORT --ids ...`) you used. Self-test output helps a lot: `python -m openxl330.selftest --sim 3 --ids 1,2,3 --out selftest-report` runs without hardware. On a bench, the same command with `--port` moves the joints, so read [the API guide](docs/api.md) first.
- **Install problems:** attach the run's `receipt.json` from `~/.openxl330/records/_runs/`. Read it first: receipts hold your servo's IDs, settings and calibration fingerprint, and the wire log next to it holds raw bus traffic. Do not attach `nvm.bin` (your servo's own calibration).
- **Security issues:** see [SECURITY.md](SECURITY.md); please do not open a public issue.

## Development

```sh
python -m pip install -e ".[dev]"
python -m pytest -q
```

The tests run on a simulated bus (`openxl330.sim`): no serial port is ever opened, and every test gets its own empty `$OPENXL330_HOME`. Add a simulated test for every change. If a change needs hardware to prove it, say so in the pull request and describe what you ran: bench, adapter, servos, firmware release and the results.

## Rules the tests enforce

- **No ROBOTIS material.** Do not add stock firmware, any bytes of the retained ROBOTIS bootloader, or tools that read them. The installer keeps only the loader's CRC-32.
- **No codec and no readout helpers.** The repository ships the pre-encoded firmware image and no encoder for it. It also has no code that recovers flash contents from loader replies. `tests/conftest.py` scans for both. The scan for the encoder's constants needs a digest list that maintainers keep outside the repository, so it runs on their machines and is skipped elsewhere.
- **No private data.** Do not commit personal names, home-directory paths, emails, adapter serial numbers, receipts, wire logs or calibration files. `tests/test_docs.py` scans every file.
- **Every installer refusal happens before the loader is entered.** Only the program region is ever written.
- **No performance-limiting guards** in the firmware or the interactive tools. Protections are what the user configures; watchdogs and leases are opt-in.

## Style

- Python 3.10+, standard library plus pyserial.
- Messages say what happened and what to do next, and every exception derives from `openxl330.Error`.
- Documentation uses plain, short sentences. Keep `docs/troubleshooting.md` covering every error class.
