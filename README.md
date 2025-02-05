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

1. `CONFIG_PTP_1588_CLOCK_ZL3073X`
This Kconfig option enables support for Precision Time Protocol (PTP) using the ZL3073X clock. PTP is used to synchronize clocks across a network with high precision, often required in telecommunications and industrial automation.
When this configuration is enabled `CONFIG_PTP_1588_CLOCK_ZL3073X`, the driver will include specific logic to interface with and manage the ZL3073X clock device, ensuring accurate time synchronization.
This config is used with the ptp4l instance is controlling/managing the ZL3037X clock.  If the ptp4l instance is controlling/managing a different hardware clock, then this is not configured. In the case of the MD-990-0011 this is not configured/used.

2. `CONFIG_DPLL`
This Kconfig option enables support for Digital Phase-Locked Loop (DPLL Manager) functionality within the driver. DPLL systems are essential for clock generation and synchronization, providing stability and jitter reduction in time-sensitive applications.
When this configuration is enabled `CONFIG_DPLL`, the driver incorporates features to interact with DPLL components, allowing fine-grained control over frequency adjustments and synchronization to a reference clock.
In the case of the MD-990-0011 this is configured/used.

3. `CONFIG_ZL3073X_MFG_FILE_AVAILABLE`
The ZL3073x may come with a pre-programmed flash configuration for compatibility with the MD-990-0011 Rev 0x0A device. However, if a user wishes to apply a custom configuration (such as PLL input and output settings) tailored to their specific hardware, they should enable `CONFIG_ZL3073X_MFG_FILE_AVAILABLE`.


4. `CONFIG_MD_990_0011_REV_0x00080000` and ` CONFIG_MD_990_0011_REV_0x000A0000` 
The user can select between `CONFIG_MD_990_0011_REV_0x00080000` and ` CONFIG_MD_990_0011_REV_0x000A0000` to configure the appropriate device revision/board pinout. The revision can be read from register 0x0007 - 0x000A. By default, both settings are disabled. To enable support for Rev 0x0A, the user must explicitly set ` CONFIG_MD_990_0011_REV_0x000A0000`, or for Rev 0x08, set `CONFIG_MD_990_0011_REV_0x00080000`.

### Build Command

```sh
make clean
make
```