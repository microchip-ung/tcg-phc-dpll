# Linux Device Driver for Microchip ZL3073X

This repository contains the source code for the ZL3073X device driver. It is organized into two different directories: one for the MFD (Multi-Function Device) driver and one for the PTP (Precision Time Protocol) driver.

## Overview

- **MFD Driver**: Used to convert from regmap reads/writes to specific I2C or SPI transactions.
- **PTP Driver**: Used to expose the PHC which can be controlled by any userspace application.

Depending on your board design, it is required to have either the `microchip-dpll-i2c` or the `microchip-dpll-spi` driver.

**Note:** The `zl3073x` naming used in the code covers all Azurite-based family of products, including ZL80132B.

**Note:** Build the code with Linux 6.11.7 or higher versions.

## How to Build

Navigate to the corresponding directory and run the following commands:

### Build Command

```sh
make clean
make
```