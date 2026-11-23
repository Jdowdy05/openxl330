# Security

openxl330 writes servo flash and drives motors, so we treat these as security issues:

- a way for openxl330 to write outside the servo's program region;
- a way for it to read out the retained ROBOTIS bootloader or stock firmware;
- a way for it to enable torque without being asked;
- private data leaking into published files, receipts or logs.

## Reporting

<!-- TODO(owner): add the project's security contact address here (a project address, not a personal one), and enable GitHub private vulnerability reporting under Settings > Security. Remove this comment before the repository goes public. -->

Report a vulnerability privately through GitHub: on the repository page, open **Security → Report a vulnerability**. Please do not open a public issue for it. Include the openxl330 version, the firmware release, and what an attacker or a mistake could cause.

We will acknowledge reports and keep you informed. This is a small volunteer project, so there is no bug bounty.

## Scope

- **In scope:** the Python package, the CLI, and the bundled firmware image.
- **Out of scope:** ROBOTIS firmware, the ROBOTIS bootloader, and Dynamixel Wizard. Report problems in those to ROBOTIS.
