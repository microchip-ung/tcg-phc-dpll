# Introduction

This document provides a detailed description of the ZL3073X driver code. The ZL3073X is a clock device driver that interfaces with the Linux kernel to provide functionalities such as reading and writing to device registers, handling PTP (Precision Time Protocol) operations, and managing DPLL (Digital Phase-Locked Loop) configurations.

To enable Azurite Dpll Netlink, turn on CONFIG_DPLL. To enable Azurite PHC, turn on CONFIG_PTP_1588_CLOCK_ZL3073X.
 
The code supports 2 board configurations; define CONFIG_MD_990_0011_REV_8 to support 
rev8 board and CONFIG_MD_990_0011_REV_A for revA board.

# Table of Contents
1. [Data Structures](#data-structures)
2. [Driver Initialization and Registration](#driver-initialization-and-registration)
3. [PTP Functions](#ptp-functions)
4. [DPLL Functions](#dpll-functions)
5. [Appendix](#appendix)


# Data Structures

This section describes the data structures used in the driver.

## Structs

- `struct zl3073x_pin`
- `struct zl3073x_dpll`
- `struct zl3073x`

## Enumerations

- `enum zl3073x_mode_t`
- `enum zl3073x_dpll_state_t`
- `enum zl3073x_tod_ctrl_cmd_t`
- `enum zl3073x_output_mode_signal_format_t`
- `enum zl3073x_pin_type`


# Driver Initialization and Registration

This section covers the initialization and registration of the driver, including initializing PTP and DPLL functionalities, and registering the driver with the platform.

## Driver Initialization

```c
static int zl3073x_probe(struct platform_device *pdev);
static void zl3073x_remove(struct platform_device *pdev);
```
- Probes and initializes the ZL3073X driver.
- Removes and cleans up the ZL3073X driver.

## DPLL Initialization

```c
static int zl3073x_dpll_init(struct zl3073x *zl3073x);
static int zl3073x_dpll_init_fine_phase_adjust(struct zl3073x *zl3073x);
```
- Initializes all DPLLs in the ZL3073X device.
- Initializes fine phase adjustment for the ZL3073X device.

## PTP Initialization

```c
static int zl3073x_ptp_init(struct zl3073x *zl3073x, u8 index);
```
- Initializes PTP functionality for a specified DPLL.

## DPLL NCO Mode Check

```c
static bool zl3073x_dpll_nco_mode(struct zl3073x *zl3073x, int dpll_index);
```
- Checks if a specified DPLL is in NCO mode.

## Platform Driver Registration

```c
static struct platform_driver zl3073x_driver = {
	.driver = {
		.name = "microchip,zl3073x-phc",
		.of_match_table = zl3073x_match,
	},
	.probe = zl3073x_probe,
	.remove	= zl3073x_remove,
};

module_platform_driver(zl3073x_driver);
```
- Registers the ZL3073X platform driver with the kernel.

## Module Information

```c
MODULE_DESCRIPTION("Driver for zl3073x clock devices");
MODULE_LICENSE("GPL");
```
- Provides a description of the ZL3073X driver module.
- Specifies the license for the ZL3073X driver module.

# PTP Functions

These functions handle PTP operations such as getting and setting time, adjusting phase, and enabling/disabling PTP outputs.

## PTP Time Operations

```c
static int zl3073x_ptp_gettime64(struct ptp_clock_info *ptp, struct timespec64 *ts);
static int zl3073x_ptp_settime64(struct ptp_clock_info *ptp, const struct timespec64 *ts);
static int zl3073x_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta);
static int zl3073x_ptp_adjfine(struct ptp_clock_info *ptp, long scaled_ppm);
static int zl3073x_ptp_adjphase(struct ptp_clock_info *ptp, s32 delta);
```
- Retrieves the current PTP time.
- Sets the PTP time.
- Adjusts the PTP time by a specified delta.
- Adjusts the PTP frequency by a scaled parts-per-million value.
- Adjusts the PTP phase by a specified delta.

## PTP Output Control

```c
static int zl3073x_ptp_perout_enable(struct zl3073x_dpll *dpll, struct ptp_perout_request *perout);
static int zl3073x_ptp_perout_disable(struct zl3073x_dpll *dpll, struct ptp_perout_request *perout);
static int zl3073x_ptp_enable(struct ptp_clock_info *ptp, struct ptp_clock_request *rq, int on);
```
- Enables a PTP periodic output.
- Disables a PTP periodic output.
- Enables or disables a PTP clock request.

## PTP Pin Verification

```c
static int zl3073x_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin, enum ptp_pin_function func, unsigned int chan);
```
- Verifies the configuration of a PTP pin.

# DPLL Functions

These functions manage DPLL configurations, including getting and setting DPLL modes, lock status, and phase offsets.

## DPLL Mode and Lock Status

```c
static int zl3073x_dpll_raw_mode_get(struct zl3073x *zl3073x, int dpll_index);
static int zl3073x_dpll_raw_lock_status_get(struct zl3073x *zl3073x, int dpll_index);
static int zl3073x_dpll_map_raw_to_manager_mode(int raw_mode);
static int zl3073x_dpll_map_raw_to_manager_lock_status(struct zl3073x *zl3073x, int dpll_index, u8 dpll_status);

```
- Retrieves the raw mode of a specified DPLL.
- Retrieves the raw lock status of a specified DPLL.
- Maps a raw DPLL mode to a manager mode.
- Maps a raw DPLL lock status to a manager lock status.

## DPLL Phase Offset

```c
static int zl3073x_dpll_phase_offset_get(struct zl3073x *zl3073x, struct zl3073x_dpll *zl3073x_dpll, struct zl3073x_pin *zl3073x_pin, s64 *phase_offset);
static int zl3073x_dpll_get_input_phase_adjust(struct zl3073x *zl3073x, u8 refId, s32 *phaseAdj);
static int zl3073x_dpll_set_input_phase_adjust(struct zl3073x *zl3073x, u8 refId, s32 phaseOffsetComp32);
static int zl3073x_dpll_get_output_phase_adjust(struct zl3073x *zl3073x, u8 outputIndex, s32 *phaseAdj)
static int zl3073x_dpll_set_output_phase_adjust(struct zl3073x *zl3073x, u8 outputIndex, s32 phaseOffsetComp32)


```
- Retrieves the phase offset of a specified DPLL pin.
- Retrieves the input phase adjustment value for a specified DPLL. 
- Sets the input phase adjustment value for a specified DPLL.
- Retrieves the current output phase adjustment value of the specified DPLL.
- Sets the output phase adjustment value of the specified DPLL.


## DPLL Pin Operations

```c
static int zl3073x_dpll_pin_frequency_set(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, const u64 frequency, struct netlink_ext_ack *extack);
static int zl3073x_dpll_pin_frequency_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, u64 *frequency, struct netlink_ext_ack *extack);
static int zl3073x_dpll_pin_direction_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, enum dpll_pin_direction *direction, struct netlink_ext_ack *extack);
static int zl3073x_dpll_pin_state_on_dpll_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, enum dpll_pin_state *state, struct netlink_ext_ack *extack);
static int zl3073x_dpll_pin_prio_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, u32 *prio, struct netlink_ext_ack *extack);
static int zl3073x_dpll_pin_prio_set(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, const u32 prio, struct netlink_ext_ack *extack);
static int zl3073x_dpll_pin_phase_adjust_set(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, const s32 phase_adjust, struct netlink_ext_ack *extack);
static int zl3073x_dpll_lock_status_get(const struct dpll_device *dpll, void *dpll_priv, enum dpll_lock_status *status, enum dpll_lock_status_error *status_error, struct netlink_ext_ack *extack);
static int zl3073x_dpll_mode_get(const struct dpll_device *dpll, void *dpll_priv, enum dpll_mode *mode, struct netlink_ext_ack *extack);
tatic int zl3073x_dpll_pin_state_on_pin_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_pin *parent_pin, void *parent_pin_priv, enum dpll_pin_state *state,
				struct netlink_ext_ack *extack);
```
- Sets the frequency of a specified DPLL pin.
- Retrieves the frequency of a specified DPLL pin.
- Retrieves the direction of a specified DPLL pin.
- Retrieves the state of a specified DPLL pin on a DPLL.
- Retrieves the priority of a specified DPLL pin.
- Sets the priority of a specified DPLL pin.
- Sets the phase adjustment of a specified DPLL pin.
- Retrieves the lock status of a specified DPLL.
- Retrieves the mode of a specified DPLL.
- Allowing software to monitor and respond to the state of various pins on the device.

## DPLL Pin I/O Operations

```c
static int zl3073x_dpll_input_pin_frequency_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, u64 *frequency, struct netlink_ext_ack *extack);
static int zl3073x_dpll_output_pin_frequency_set(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, const u64 frequency, struct netlink_ext_ack *extack);
static int zl3073x_dpll_input_pin_phase_offset_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, s64 *phase_offset, struct netlink_ext_ack *extack);
static int zl3073x_dpll_input_pin_phase_adjust_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, s32 *phase_adjust, struct netlink_ext_ack *extack);
static int zl3073x_dpll_output_pin_phase_adjust_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, s32 *phase_adjust, struct netlink_ext_ack *extack);
static int zl3073x_dpll_output_pin_phase_adjust_set(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, const s32 phase_adjust, struct netlink_ext_ack *extack);
static int zl3073x_dpll_input_pin_ffo_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, s64 *ffo, struct netlink_ext_ack *extack);
static int zl3073x_dpll_input_pin_esync_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, struct dpll_pin_esync *esync, struct netlink_ext_ack *extack);
static int zl3073x_dpll_input_pin_esync_set(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, u64 freq, struct netlink_ext_ack *extack);
static int zl3073x_dpll_output_pin_esync_get(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, struct dpll_pin_esync *esync, struct netlink_ext_ack *extack);
static int zl3073x_dpll_output_pin_esync_set(const struct dpll_pin *pin, void *pin_priv, const struct dpll_device *dpll, void *dpll_priv, u64 freq, struct netlink_ext_ack *extack);
```
- Retrieves the input frequency of a specifc pin.
- Retrieves the output frequency of a specific pin.
- Retrieves input phase offset of a specific pin.
- Retrieves the input phase adjustment of a specific pin.
- Retrieves the output phase adjustment of a specific pin.
- Sets the output phase adjustment value of a specific pin.
- Retrieves the input ffo of a specific pin.
- Retrieves the esync settings at a specific input pin. 
- Sets the esync setting of a specific input pin.
- Retrieves the esync settings of a specified output pin.
- Sets the esync settings of a specified output pin.









# Appendix
This section has extra driver information and unility functions

## Constants

This section defines various constants used throughout the driver code.

- `ZL3073X_1PPM_FORMAT`
- `ZL3073X_MAX_SYNTH`
- `ZL3073X_MAX_INPUT_PINS`
- `ZL3073X_MAX_OUTPUT_PINS`
- `ZL3073X_MAX_OUTPUT_PIN_PAIRS`
- `ZL3073X_MAX_DPLLS`
- `ZL3073X_MAX_PINS`
- `READ_SLEEP_US`
- `READ_TIMEOUT_US`
- `ZL3073X_FW_FILENAME`
- `ZL3073X_FW_WHITESPACES_SIZE`
- `ZL3073X_FW_COMMAND_SIZE`

## Utility Functions

These functions provide basic utilities such as reading and writing to device registers and converting between different data formats.

### Byte Swapping

```c
static u8 *zl3073x_swap(u8 *swap, u16 count);
```
- Swaps the bytes in an array to match the DPLL register format.

### Register Read/Write

```c
static int zl3073x_read(struct zl3073x *zl3073x, u16 regaddr, u8 *buf, u16 count);
static int zl3073x_write(struct zl3073x *zl3073x, u16 regaddr, u8 *buf, u16 count);
```
- Reads a block of data from the specified register address.
- Writes a block of data to the specified register address.

### Timestamp Conversion

```c
static void zl3073x_ptp_timestamp_to_bytearray(const struct timespec64 *ts, u8 *sec, u8 *nsec);
static void zl3073x_ptp_bytearray_to_timestamp(struct timespec64 *ts, u8 *sec, u8 *nsec);
```
- Converts a timespec64 timestamp to a byte array.
- Converts a byte array to a timespec64 timestamp.