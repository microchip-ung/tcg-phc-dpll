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

## KConfig setting

1. CONFIG_PTP_1588_CLOCK_ZL3073X

This Kconfig option enables support for Precision Time Protocol (PTP) using the ZL3073X clock. PTP is used to synchronize clocks across a network with high precision, often required in telecommunications and industrial automation.

When this configuration is enabled (IS_ENABLED(CONFIG_PTP_1588_CLOCK_ZL3073X)), the driver will include specific logic to interface with and manage the ZL3073X clock device, ensuring accurate time synchronization.

2. CONFIG_DPLL

This Kconfig option enables support for Digital Phase-Locked Loop (DPLL Manager) functionality within the driver. DPLL systems are essential for clock generation and synchronization, providing stability and jitter reduction in time-sensitive applications.

When this configuration is enabled (IS_ENABLED(CONFIG_DPLL)), the driver incorporates features to interact with DPLL components, allowing fine-grained control over frequency adjustments and synchronization to a reference clock.

### Build Command

```sh
make clean
make
```