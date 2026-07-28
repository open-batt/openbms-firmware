# OpenBMS Firmware

![Firmware](https://img.shields.io/badge/firmware-STM32-03234B)
![License](https://img.shields.io/badge/license-MIT-7c5cbf)

[OpenBMS](https://github.com/open-batt/openbms-hardware) firmware runs on STM32L4 microcontroller and is written in bare C. It consists of two layers:
- Peripheral control and communication protocol handler - initializes and runs all peripherals on STM32L4, reads ADC, controls GPIO, handles communication on I2C, UART and CAN.
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
- Always open this project via STM32CubeIDE or the extension so `CUBE_BUNDLE_PATH` is set correctly
- Check that bundle version numbers in `.vscode/settings.json` and `.vscode/launch.json` match your locally installed versions under `%LOCALAPPDATA%\stm32cube\bundles\`

## 📡 Communication Protocol

OpenBMS implements three communication protocols sharing the same register map:

- **SBS v1.1 over I²C/SMBus** — Smart Battery Specification v1.1, compatible with any SBS-compliant host. SMBus address `0x0B`, speed up to 100 kHz, PEC error checking.
- **Modbus RTU over UART** — 115200 baud, 8N1, same register addresses as SBS. Used for debugging and external communication.
- **CAN 2.0** — 500 kbit/s, 11-bit identifier, same register addresses as SBS. Used for communication with host, charger and other system components.

Full protocol documentation can be found in [communication-protocol.md](communication-protocol.md).
