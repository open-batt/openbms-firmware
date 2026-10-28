# OpenBMS Firmware

![Firmware](https://img.shields.io/badge/firmware-STM32-03234B)
![License](https://img.shields.io/badge/license-MIT-7c5cbf)

[OpenBMS](https://github.com/open-batt/openbms-hardware) firmware runs on STM32L4 microcontroller and is written in bare C. It consists of two layers:
- Peripheral control and communication protocol handler - initializes and runs all peripherals on STM32L4, reads ADC, controls GPIO, and implements the host communication protocol over UART and I2C (slave `0x0B`, same frames as UART), plus a CAN status broadcast and short CAN commands — see [communication-protocol.md](communication-protocol.md).
- Algorithms - on-board HPPC test with battery model parameter extraction, battery fuel gauge (SOC, capacity learning, SOH, temperature compensation), cell balancing control, protections.

## ❤️ Funding

This project is funded through [NGI0 Commons Fund](https://nlnet.nl/commonsfund), a fund established by [NLnet](https://nlnet.nl) with financial support from the European Commission's [Next Generation Internet](https://ngi.eu) program. 

[<img src="https://nlnet.nl/logo/banner.png" alt="NLnet foundation logo" width="20%" />](https://nlnet.nl)
[<img src="https://nlnet.nl/image/logos/NGI0_tag.svg" alt="NGI Zero Logo" width="20%" />](https://nlnet.nl/commonsfund)

We are very grateful to the NLnet team for helping us on our path, and we encourage you too to apply and get funds to build your project! 🚀
Learn more at the [NLnet project page](https://nlnet.nl/project/OpenBMS).

## ⚠️ Status: Work in progress

| Module | Status |
|--------|--------|
| Communication Protocol Define | ✅ Done |
| Fuel Gauge Algorithms | ✅ Done |
| Implementation on STM32 | ✅ Done |
| Test & Bug Fix | 🚧 In progress |

## Prerequisites
- Install STM32CubeIDE for VS Code extension
- Check that bundle version numbers in `firmware/.vscode/settings.json` and `firmware/.vscode/launch.json` match your locally installed versions under `%LOCALAPPDATA%\stm32cube\bundles\`
- Build with the **Release** preset (the only one, `-Os`); the output is in `firmware/build/Release`

## 📚 Documentation

- **[how-to-use.md](how-to-use.md)** — end-to-end walkthrough: first-time setup, flashing, writing the config, running the on-board HPPC test, using the BMS, and testing against a simulated battery.
- **[battery-model.md](battery-model.md)** — 2-RC equivalent circuit battery model, on-board HPPC parameter extraction, SOC estimation via an Extended Kalman Filter with its corrections, capacity learning and temperature compensation, with a worked 7-cell hardware example.
- **[communication-protocol.md](communication-protocol.md)** — full wire protocol: the OpenBMS binary protocol over UART and I²C, the CAN status broadcast and commands, and the complete register map.
- **[python-script.md](python-script.md)** — reference for the host-side scripts in `python_scripts/` (flashing firmware, sending commands, writing the config, running the HPPC test, simulation tests).

## 🔗 Connected Projects

- **[openbms-hardware](https://github.com/open-batt/openbms-hardware)** — the PCB and hardware design this firmware runs on.
- **[openbms-test-bench-hardware](https://github.com/open-batt/openbms-test-bench-hardware)** — the test bench hardware used for HPPC characterization and validation.
- **[openbms-studio](https://github.com/open-batt/openbms-studio)** — the desktop app for configuring and monitoring the board (in progress).