# OpenBMS Firmware

![Firmware](https://img.shields.io/badge/firmware-STM32-03234B)
![License](https://img.shields.io/badge/license-MIT-7c5cbf)

[OpenBMS](https://github.com/open-batt/openbms-hardware) firmware runs on STM32L4 microcontroller and is written in bare C. It consists of two layers:
- Peripheral control and communication protocol handler - initializes and runs all peripherals on STM32L4, reads ADC, controls GPIO, and implements the host communication protocol over UART (I2C and CAN peripherals are initialized but not yet handled — see [communication-protocol.md](communication-protocol.md)).
- Algorithms - battery fuel gauge algorithm, charge, discharge and cell balancing control, protections.

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
| Fuel Gauge Algorithms | 🔜 Planned |
| Implementation on STM32 | ❌ Not started |
| Test & Bug Fix | ❌ Not started |

## Prerequisites
- Install STM32CubeIDE for VS Code extension
- Check that bundle version numbers in `firmware/.vscode/settings.json` and `firmware/.vscode/launch.json` match your locally installed versions under `%LOCALAPPDATA%\stm32cube\bundles\`

## 📚 Documentation

- **[communication-protocol.md](communication-protocol.md)** — full wire protocol. The OpenBMS binary protocol over UART is implemented; I²C and CAN 2.0 are planned (draft frame formats included).
- **[battery-model.md](battery-model.md)** — 2-RC equivalent circuit battery model, HPPC parameter extraction, and SOC estimation via an Extended Kalman Filter.
- **[python-script.md](python-script.md)** — reference for the host-side scripts in `python_scripts/` (flashing firmware, running hardware tests, analyzing logs).
