// SPDX-License-Identifier: GPL-2.0

#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/timekeeping.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/mfd/microchip-dpll.h>
#include <linux/regmap.h>
#include <linux/dpll.h>

#include "ptp_private.h"

#define DPLL_CHIP_ID_REG				(0x01)

#define DPLL_REF_MON_STATUS(index)			(0x102 + (index))
#define DPLL_REF_MON_STATUS_QUALIFIED(val)	(!val)

#define DPLL_MON_STATUS(index)			(0x110 + (index))
#define DPLL_MON_STATUS_HO_READY_GET(val)	((val & GENMASK(2, 2)) >> 2)
#define DPLL_LOCK_REFSEL_STATUS(index)		(0x130 + (index))
#define DPLL_LOCK_REFSEL_LOCK_GET(val)		((val & GENMASK(6, 4)) >> 4)
#define DPLL_LOCK_REFSEL_REF_GET(val)		((val & GENMASK(3, 0)))

#define DPLL_REF_FREQ_ERR(ref)			(0x144 + (ref) * 0x4)

#define DPLL_REF_PHASE_ERR_RQST			(0x20f)
#define DPLL_REF_PHASE_ERR_RQST_MASK	GENMASK(0, 0)

#define REF_FREQ_MEAS_CTRL				0x21C
#define REF_FREQ_MEAS_CTRL_MASK			GENMASK(1, 0)

#define REF_FREQ_MEAS_MASK_3_0			0x21D
#define REF_FREQ_MEAS_MASK_4			0x21E

#define DPLL_MEAS_REF_FREQ_CTRL			0x21F
#define DPLL_MEAS_REF_FREQ_MASK_SHIFT	4

#define DPLL_REF_PHASE_ERR(ref)			(0x220 + (ref) * 0x6)

#define DPLL_MODE_REFSEL(index)			(0x284 + (index) * 0x4)
#define DPLL_MODE_REFSEL_MODE_GET(val)		(val & GENMASK(2, 0))
#define DPLL_MODE_REFSEL_REF_GET(val)		(val & GENMASK(7, 4))

#define DPLL_TIE_CTRL				0x2b0
#define DPLL_TIE_CTRL_MASK			GENMASK(2, 0)
#define DPLL_TIE_CTRL_MASK_REG			0x2b1
#define DPLL_TIE_CTRL_OPERATION			4
#define DPLL_TIE_CTRL_SIZE			1

#define DPLL_MEAS_CTRL				(0x2D0)
#define DPLL_MEAS_CTRL_EN_MASK		GENMASK(0, 0)
#define DPLL_MEAS_IDX_REG			(0x2D1)
#define DPLL_MEAS_IDX_MASK			GENMASK(2, 0)

#define DPLL_SYNTH_CTRL(index)			(0x480 + (index))
#define DPLL_SYNTH_CTRL_DPLL_SEL_GET(val)	((val & GENMASK(6, 4)) >> 4)

#define DPLL_TOD_CTRL(index)			(0x2b8 + (index))
#define DPLL_TOD_CTRL_SEM			BIT(4)

#define DPLL_DF_OFFSET(index)			(0x300 + (index) * 0x20)
#define DPLL_TIE_DATA(index)			(0x30c + (index) * 0x20)
#define DPLL_TOD_SEC(index)			(0x312 + (index) * 0x20)
#define DPLL_TOD_SEC_SIZE			6
#define DPLL_TOD_NSEC(index)			(0x318 + (index) * 0x20)
#define DPLL_TOD_NSEC_SIZE			6

#define DPLL_SYNTH_PHASE_SHIFT_CTRL		0x49e
#define DPLL_SYNTH_PHASE_SHIFT_MASK		0x49f
#define DPLL_SYNTH_PHASE_SHIFT_INTVL		0x4a0
#define DPLL_SYNTH_PHASE_SHIFT_DATA		0x4a1

#define DPLL_OUTPUT_CTRL(index)			(0x4a8 + (index))
#define DPLL_OUTPUT_CTRL_SIZE			1
#define DPLL_OUTPUT_CTRL_SYNTH_SEL_GET(val)	((val & GENMASK(6, 4)) >> 4)
#define DPLL_OUTPUT_CTRL_STOP			BIT(1)
#define DPLL_OUTPUT_CTRL_STOP_HIGH		BIT(2)
#define DPLL_OUTPUT_CTRL_STOP_HZ		BIT(3)

#define DPLL_OUTPUT_PHASE_STEP_CTRL		0x4b8
#define DPLL_OUTPUT_PHASE_STEP_CTRL_SIZE	1
#define DPLL_OUTPUT_PHASE_STEP_CTRL_OP(cmd)	(cmd & GENMASK(1, 0))
#define DPLL_OUTPUT_PAHSE_STEP_CTRL_OP_WRITE	3
#define DPLL_OUTPUT_PHASE_STEP_CTRL_OP_MASK	GENMASK(1, 0)
#define DPLL_OUTPUT_PHASE_STEP_CTRL_TOD_STEP	BIT(3)
#define DPLL_OUTPUT_PHASE_STEP_CTRL_DPLL(index)	((index) << 4)
#define DPLL_OUTPUT_PHASE_STEP_NUMBER		0x4b9
#define DPLL_OUTPUT_PHASE_STEP_NUMBER_SIZE	1
#define DPLL_OUTPUT_PHASE_STEP_MASK		0x4ba
#define DPLL_OUTPUT_PHASE_STEP_MASK_SIZE	2
#define DPLL_OUTPUT_PHASE_STEP_DATA		0x4bc
#define DPLL_OUTPUT_PHASE_STEP_DATA_SIZE	4

#define DPLL_REF_MB_MASK								0x502
#define DPLL_REF_MB_MASK_SIZE								2
#define DPLL_REF_MB_SEM									0x504
#define DPLL_REF_MB_SEM_SIZE								1
#define DPLL_REF_MB_SEM_RD								BIT(1)
#define DPLL_REF_MB_SEM_WR								BIT(0)
#define DPLL_REF_PHASE_OFFSET_COMPENSATION_REG			0x528
#define DPLL_REF_PHASE_OFFSET_COMPENSATION_REG_SIZE			6

#define DPLL_DPLL_MB_MASK			0x602
#define DPLL_DPLL_MB_MASK_SIZE			2
#define DPLL_DPLL_MB_SEM			0x604
#define DPLL_DPLL_MB_SEM_SIZE			1
#define DPLL_DPLL_MB_SEM_RD			BIT(1)
#define DPLL_DPLL_MB_SEM_WR			BIT(0)

#define DPLL_REF_PRIORITY(refId)				(0x652 + (refId/2))
#define DPLL_REF_PRIORITY_GET_UPPER(data)		(((data) & GENMASK(7, 4)) >> 4)
#define DPLL_REF_PRIORITY_GET_LOWER(data)		((data) & GENMASK(3, 0))

#define DPLL_REF_PRIORITY_GET(data, refId) \
	(((refId) % 2 == 0) ? DPLL_REF_PRIORITY_GET_LOWER(data) : \
							DPLL_REF_PRIORITY_GET_UPPER(data))

#define DPLL_REF_PRIORITY_SET_LOWER(data, value) \
	(((data) & GENMASK(7, 4)) | ((value) & GENMASK(3, 0)))

#define DPLL_REF_PRIORITY_SET_UPPER(data, value) \
	(((data) & GENMASK(3, 0)) | (((value) & GENMASK(3, 0)) << 4))

#define DPLL_REF_PRIORITY_SET(data, refId, value) \
	(((refId) % 2 == 0) ? DPLL_REF_PRIORITY_SET_LOWER((data), (value)) : \
							DPLL_REF_PRIORITY_SET_UPPER((data), (value)))

#define DPLL_REF_PRIORITY_INVALID	0xf

#define DPLL_SYNTH_MB_MASK			0x682
#define DPLL_SYNTH_MB_MASK_SIZE			2
#define DPLL_SYNTH_MB_SEM			0x684
#define DPLL_SYNTH_MB_SEM_SIZE			1
#define DPLL_SYNTH_MB_SEM_RD			BIT(1)
#define DPLL_SYNTH_FREQ_BASE			0x686
#define DPLL_SYNTH_FREQ_BASE_SIZE		2
#define DPLL_SYNTH_FREQ_MULT			0x688
#define DPLL_SYNTH_FREQ_MULT_SIZE		4
#define DPLL_SYNTH_FREQ_M			0x68c
#define DPLL_SYNTH_FREQ_M_SIZE			2
#define DPLL_SYNTH_FREQ_N			0x68e
#define DPLL_SYNTH_FREQ_N_SIZE			2

#define DPLL_OUTPUT_MB_MASK			0x702
#define DPLL_OUTPUT_MB_MASK_SIZE		2
#define DPLL_OUTPUT_MB_SEM			0x704
#define DPLL_OUTPUT_MB_SEM_SIZE			1
#define DPLL_OUTPUT_MB_SEM_RD			BIT(1)
#define DPLL_OUTPUT_MB_SEM_WR			BIT(0)
#define DPLL_OUTPUT_MODE			0x705
#define DPLL_OUTPUT_MODE_SIZE			1
#define DPLL_OUTPUT_MODE_SIGNAL_FORMAT(val)	((val) << 4)
#define DPLL_OUTPUT_MODE_SIGNAL_FORMAT_GET(val) ((val & GENMASK(7, 4)) >> 4)
#define DPLL_OUTPUT_MODE_SIGNAL_FORMAT_MASK	GENMASK(7, 4)
#define DPLL_OUTPUT_DIV				0x70c
#define DPLL_OUTPUT_DIV_SIZE			4
#define DPLL_OUTPUT_WIDTH			0x710
#define DPLL_OUTPUT_WIDTH_SIZE			4
#define DPLL_OUTPUT_PHASE_COMPENSATION_REG			0x720
#define DPLL_OUTPUT_PHASE_COMPENSATION_REG_SIZE			4
#define DPLL_OUTPUT_GPO_EN			0x724
#define DPLL_OUTPUT_GPO_EN_SIZE			1

#define ZL3073X_1PPM_FORMAT		281474976

#define ZL3073X_MAX_SYNTH		5
#define ZL3073X_MAX_INPUT_PINS			10
#define ZL3073X_MAX_OUTPUT_PINS		20
#define ZL3073X_MAX_OUTPUT_PIN_PAIRS	(ZL3073X_MAX_OUTPUT_PINS / 2)
#define ZL3073X_MAX_DPLLS		2
#define ZL3073X_MAX_PINS		(ZL3073X_MAX_INPUT_PINS + ZL3073X_MAX_OUTPUT_PINS)

#define ZL3073X_PTP_CLOCK_DPLL	0

#define READ_SLEEP_US			10
#define READ_TIMEOUT_US			100000000

#define ZL3073X_FW_FILENAME		"zl3073x.mfg"
#define ZL3073X_FW_WHITESPACES_SIZE	3
#define ZL3073X_FW_COMMAND_SIZE		1

#define ZL3073X_P_PIN(pin)		((pin) % 2 == 0)
#define ZL3073X_N_PIN(pin)		(!ZL3073X_P_PIN(pin))

#define ZL3073X_IS_INPUT_PIN(pin)	(pin >= 20 && pin < 30)
#define ZL3073X_IS_OUTPUT_PIN(pin)	(!ZL3073X_IS_INPUT_PIN(pin))

#define ZL3073X_REG_MAP_INPUT_PIN_GET(pin)	(pin - 20)

#define ZL3073X_CHECK_REF_ID(ref)		((ref >= 0) && (ref < ZL3073X_MAX_INPUT_PINS))
#define ZL3073X_CHECK_OUTPUT_ID(output)	((output >= 0) && (output < ZL3073X_MAX_OUTPUT_PINS))
#define ZL3073X_CHECK_SYNTH_ID(synth)	((synth >= 0) && (synth < ZL3073X_MAX_SYNTH))

#define MD_990_0011

static const struct of_device_id zl3073x_match[] = {
	{ .compatible = "microchip,zl80732" },
	{ .compatible = "microchip,zl30732b" },
	{ }
};
MODULE_DEVICE_TABLE(of, zl3073x_match);

enum zl3073x_mode_t {
	ZL3073X_MODE_FREERUN        = 0x0,
	ZL3073X_MODE_HOLDOVER       = 0x1,
	ZL3073X_MODE_REFLOCK        = 0x2,
	ZL3073X_MODE_AUTO_LOCK      = 0x3,
	ZL3073X_MODE_NCO			= 0x4,
};

enum zl3073x_dpll_state_t {
	ZLS3073X_DPLL_STATE_FREERUN		= 0x0,
	ZLS3073X_DPLL_STATE_HOLDOVER	= 0x1,
	ZLS3073X_DPLL_STATE_FAST_LOCK	= 0x2,
	ZLS3073X_DPLL_STATE_ACQUIRING	= 0x3,
	ZLS3073X_DPLL_STATE_LOCK		= 0x4,
};

enum zl3073x_tod_ctrl_cmd_t {
	ZL3073X_TOD_CTRL_CMD_WRITE_NEXT_1HZ	= 0x1,
	ZL3073X_TOD_CTRL_CMD_READ		= 0x8,
	ZL3073X_TOD_CTRL_CMD_READ_NEXT_1HZ	= 0x9,
};

enum zl3073x_output_mode_signal_format_t {
	ZL3073X_BOTH_DISABLED			= 0x0,
	ZL3073X_BOTH_ENABLED			= 0x4,
	ZL3073X_P_ENABLE			= 0x5,
	ZL3073X_N_ENABLE			= 0x6,
};

enum zl3073x_pin_type {
	ZL3073X_SINGLE_ENDED,
	ZL3073X_DIFFERENTIAL,
};

enum zl3073x_output_freq_type_t {
	ZL3073X_SYNCE,
	ZL3073X_PTP,
	ZL3073X_25MHz,
};

#ifdef MD_990_0011
enum dpll_type zl3073x_dpll_type[ZL3073X_MAX_DPLLS] = {
	DPLL_TYPE_EEC,
	DPLL_TYPE_PPS
};
enum zl3073x_pin_type zl3073x_output_pin_type[ZL3073X_MAX_OUTPUT_PIN_PAIRS] = {
	ZL3073X_SINGLE_ENDED,
	ZL3073X_SINGLE_ENDED,
	ZL3073X_DIFFERENTIAL,
	ZL3073X_DIFFERENTIAL,
	ZL3073X_DIFFERENTIAL,
	ZL3073X_DIFFERENTIAL,
	ZL3073X_SINGLE_ENDED,
	ZL3073X_SINGLE_ENDED,
	ZL3073X_SINGLE_ENDED,
	ZL3073X_DIFFERENTIAL,
};

struct dpll_pin_frequency input_freq_ranges[] = {
	{ .min = 1, .max = 1, },
	{ .min = 25, .max = 25, },
	{ .min = 100, .max = 100, },
	{ .min = 1000, .max = 1000, },
	{ .min = 10000000, .max = 10000000, },
	{ .min = 25000000, .max = 25000000, },
	{ .min = 62500000, .max = 62500000, },
	{ .min = 78125000, .max = 78125000, },
	{ .min = 100000000, .max = 100000000, },
};
struct dpll_pin_frequency output_freq_range_ptp[] = {
	{ .min = 1, .max = 1, },
	{ .min = 25, .max = 25, },
	{ .min = 100, .max = 100, },
	{ .min = 1000, .max = 1000, },
	{ .min = 10000000, .max = 10000000, },
	{ .min = 25000000, .max = 25000000, },
};
struct dpll_pin_frequency output_freq_range_synce[] = {
	{ .min = 156250000, .max = 156250000, },
};

struct dpll_pin_frequency output_freq_range_25MHz[] = {
	{ .min = 25000000, .max = 25000000, },
};

enum zl3073x_output_freq_type_t output_freq_type_per_output[] = {
	ZL3073X_PTP,	/* OUT0 */
	ZL3073X_PTP,	/* OUT1 */
	ZL3073X_PTP,	/* OUT2 */
	ZL3073X_SYNCE,	/* OUT3 - fixed to 156.25 Mhz */
	ZL3073X_SYNCE,	/* OUT4 - fixed to 156.25 Mhz */
	ZL3073X_SYNCE,	/* OUT5 - fixed to 156.25 Mhz */
	ZL3073X_PTP,	/* OUT6 */
	ZL3073X_PTP,	/* OUT7 */
	ZL3073X_PTP,	/* OUT8 */
	ZL3073X_25MHz,	/* OUT9 - fixed to 25 MHz */
};

/* phase adjust is stored in a 32 bit register in units of 2.5 ns
 * so just return the highest allowed by the size of the phase
 * adjust range structure (i.e signed 32 bit integer)
 */
struct dpll_pin_phase_adjust_range phase_range = {
	.min = (-2147483648),
	.max = (2147483647),
};

enum dpll_pin_type input_dpll_pin_types[] = {
	DPLL_PIN_TYPE_GNSS,				/* REF0P */
	DPLL_PIN_TYPE_GNSS,				/* REF0N */
	DPLL_PIN_TYPE_SYNCE_ETH_PORT,	/* REF1P */
	DPLL_PIN_TYPE_SYNCE_ETH_PORT,	/* REF1N */
	DPLL_PIN_TYPE_EXT,				/* REF2P */
	DPLL_PIN_TYPE_GNSS,				/* REF2N */
	DPLL_PIN_TYPE_EXT,				/* REF3P */
	DPLL_PIN_TYPE_EXT,				/* REF3N */
	DPLL_PIN_TYPE_GNSS,				/* REF4P */
	DPLL_PIN_TYPE_INT_OSCILLATOR,	/* REF4N */
};

enum dpll_pin_type output_dpll_pin_types[] = {
	DPLL_PIN_TYPE_GNSS,				/* OUT0P */
	DPLL_PIN_TYPE_GNSS,				/* OUT0N */
	DPLL_PIN_TYPE_GNSS,				/* OUT1P */
	DPLL_PIN_TYPE_GNSS,				/* OUT1N */
	DPLL_PIN_TYPE_GNSS,				/* OUT2P */
	DPLL_PIN_TYPE_GNSS,				/* OUT2N */
	DPLL_PIN_TYPE_SYNCE_ETH_PORT,	/* OUT3P */
	DPLL_PIN_TYPE_SYNCE_ETH_PORT,	/* OUT3N */
	DPLL_PIN_TYPE_SYNCE_ETH_PORT,	/* OUT4P */
	DPLL_PIN_TYPE_SYNCE_ETH_PORT,	/* OUT4N */
	DPLL_PIN_TYPE_SYNCE_ETH_PORT,	/* OUT5P */
	DPLL_PIN_TYPE_SYNCE_ETH_PORT,	/* OUT5N */
	DPLL_PIN_TYPE_GNSS,				/* OUT6P */
	DPLL_PIN_TYPE_INT_OSCILLATOR,	/* OUT6N */
	DPLL_PIN_TYPE_GNSS,				/* OUT7P */
	DPLL_PIN_TYPE_GNSS,				/* OUT7N */
	DPLL_PIN_TYPE_GNSS,				/* OUT8P */
	DPLL_PIN_TYPE_GNSS,				/* OUT8N */
	DPLL_PIN_TYPE_GNSS,				/* OUT9P */
	DPLL_PIN_TYPE_GNSS,				/* OUT9N */
};

const char *input_pin_names[] = {
	"1PPS_IN1",		"1PPS_IN0",
	"RCLKA_IN",		"RCLKB_IN",
	"REF2P",		"GNSS_10M_IN",
	"SMA1_IN",		"SMA3_IN",
	"GNSS_1PPS_IN",	"REF4N",
};

const char *output_pin_names[] = {
	"SMA0_OUT",		"1PPS_OUT4",
	"OUT1P",		"AIC_SCLK",
	"AIC_DCLK_P",	"AIC_DCLK_N",
	"SYNC_CLK1_P",	"SYNC_CLK1_N",
	"SYNC_CLK0_P",	"SYNC_CLK0_N",
	"SYNC_CLK2_P",	"SYNC_CLK2_N",
	"SMA2_OUT",		"SYNC_CLK_GD",
	"1PPS_OUT3",	"1PPS_OUT2",
	"1PPS_OUT1",	"1PPS_OUT0",
	"SYNC_25M_P",	"SYNC_25M_N",
};
#else
const char *input_pin_names[] = {
	"REF0P", "REF0N",
	"REF1P", "REF1N",
	"REF2P", "REF2N",
	"REF3P", "REF3N",
	"REF4P", "REF4N",
};

const char *output_pin_names[] = {
	"OUT0P", "OUT0N",
	"OUT1P", "OUT1N",
	"OUT2P", "OUT2N",
	"OUT3P", "OUT3N",
	"OUT4P", "OUT4N",
	"OUT5P", "OUT5N",
	"OUT6P", "OUT6N",
	"OUT7P", "OUT7N",
	"OUT8P", "OUT8N",
	"OUT9P", "OUT9N",
};
#endif

struct zl3073x_pin {
	struct zl3073x		*zl3073x;
	u8			index;
	enum zl3073x_pin_type pin_type;

	struct dpll_pin	*dpll_pin;
};

struct zl3073x_dpll {
	struct zl3073x		*zl3073x;
	u8			index;

	struct ptp_clock_info	info;
	struct ptp_clock	*clock;
	struct ptp_pin_desc	pins[ZL3073X_MAX_PINS];

	u16			perout_mask;
	struct dpll_device	*dpll_device;
};

struct zl3073x {
	struct device		*dev;
	struct mutex		*lock;
	struct regmap		*regmap;
	struct device		*mfd;

	struct zl3073x_dpll	dpll[ZL3073X_MAX_DPLLS];
	struct zl3073x_pin pin[ZL3073X_MAX_PINS];
};

/* When accessing the registers of the DPLL, it is always required to access
 * first the lower address and then the higher address. The MSB of the data is
 * always stored at the lowest address and LSB is stored at the highest address.
 * This format is different than most setups therefore make sure to swap the
 * bytes before writting and after reading so it can be easier to follow the
 * datasheet.  This function was added for this purpose and it is used inside
 * the read and write functions.
 */
static u8 *zl3073x_swap(u8 *swap, u16 count)
{
	int i;

	for (i = 0; i < count / 2; ++i) {
		u8 tmp = swap[i];
		swap[i] = swap[count - i - 1];
		swap[count - i - 1] = tmp;
	}

	return swap;
}

static int zl3073x_read(struct zl3073x *zl3073x, u16 regaddr, u8 *buf, u16 count)
{
	return regmap_bulk_read(zl3073x->regmap, regaddr, buf, count);
}

static int zl3073x_write(struct zl3073x *zl3073x, u16 regaddr, u8 *buf, u16 count)
{
	return regmap_bulk_write(zl3073x->regmap, regaddr, zl3073x_swap(buf, count), count);
}

static void zl3073x_ptp_timestamp_to_bytearray(const struct timespec64 *ts,
					       u8 *sec, u8 *nsec)
{
	sec[0] = (ts->tv_sec >> 0) & 0xff;
	sec[1] = (ts->tv_sec >> 8) & 0xff;
	sec[2] = (ts->tv_sec >> 16) & 0xff;
	sec[3] = (ts->tv_sec >> 24) & 0xff;
	sec[4] = (ts->tv_sec >> 32) & 0xff;
	sec[5] = (ts->tv_sec >> 40) & 0xff;

	nsec[0] = (ts->tv_nsec >> 0) & 0xff;
	nsec[1] = (ts->tv_nsec >> 8) & 0xff;
	nsec[2] = (ts->tv_nsec >> 16) & 0xff;
	nsec[3] = (ts->tv_nsec >> 24) & 0xff;
	nsec[4] = 0;
	nsec[5] = 0;
}

static void zl3073x_ptp_bytearray_to_timestamp(struct timespec64 *ts,
					       u8 *sec, u8 *nsec)
{
	ts->tv_sec = sec[0];
	for (int i = 1; i < DPLL_TOD_SEC_SIZE; ++i) {
		ts->tv_sec = ts->tv_sec << 8;
		ts->tv_sec |= sec[i];
	}

	ts->tv_nsec = nsec[0];
	for (int i = 1; i < DPLL_TOD_NSEC_SIZE - 2; ++i) {
		ts->tv_nsec = ts->tv_nsec << 8;
		ts->tv_nsec |= nsec[i];
	}

	set_normalized_timespec64(ts, ts->tv_sec, ts->tv_nsec);
}

static int zl3073x_ptp_tod_sem(struct zl3073x_dpll *dpll)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	u8 sem;

	zl3073x_read(zl3073x, DPLL_TOD_CTRL(dpll->index), &sem, sizeof(sem));
	return sem;
}

static int zl3073x_ptp_phase_ctrl_op(struct zl3073x_dpll *dpll)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	u8 ctrl;

	zl3073x_read(zl3073x, DPLL_OUTPUT_PHASE_STEP_CTRL, &ctrl, sizeof(ctrl));
	return ctrl;
}

static int zl3073x_ptp_tie_ctrl_op(struct zl3073x_dpll *dpll)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	u8 ctrl;

	zl3073x_read(zl3073x, DPLL_TIE_CTRL, &ctrl, sizeof(ctrl));
	return ctrl;
}

static int zl3073x_ptp_synth_mb_sem(struct zl3073x_dpll *dpll)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	u8 sem;

	zl3073x_read(zl3073x, DPLL_SYNTH_MB_SEM, &sem, sizeof(sem));
	return sem;
}

static int zl3073x_dpll_mb_sem(struct zl3073x *zl3073x)
{
	u8 sem;

	zl3073x_read(zl3073x, DPLL_DPLL_MB_SEM, &sem, sizeof(sem));
	return sem;
}

static int zl3073x_ptp_output_mb_sem(struct zl3073x_dpll *dpll)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	u8 sem;

	zl3073x_read(zl3073x, DPLL_OUTPUT_MB_SEM, &sem, sizeof(sem));
	return sem;
}

static int zl3073x_synth_get(struct zl3073x *zl3073x, int output_index)
{
	u8 output_ctrl;
	zl3073x_read(zl3073x, DPLL_OUTPUT_CTRL(output_index), &output_ctrl,
		     DPLL_OUTPUT_CTRL_SIZE);

	return DPLL_OUTPUT_CTRL_SYNTH_SEL_GET(output_ctrl);
}

static int _zl3073x_ptp_gettime64(struct zl3073x_dpll *dpll,
				  struct timespec64 *ts,
				  enum zl3073x_tod_ctrl_cmd_t cmd)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	u8 nsec[DPLL_TOD_NSEC_SIZE];
	u8 sec[DPLL_TOD_SEC_SIZE];
	int ret;
	u32 val;
	u8 ctrl;

	/* Check that the semaphore is clear */
	ret = readx_poll_timeout_atomic(zl3073x_ptp_tod_sem, dpll,
					val, !(DPLL_TOD_CTRL_SEM & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		return ret;

	/* Issue the read command */
	ctrl = DPLL_TOD_CTRL_SEM | cmd;
	zl3073x_write(zl3073x, DPLL_TOD_CTRL(dpll->index), &ctrl, sizeof(ctrl));

	/* Check that the semaphore is clear */
	ret = readx_poll_timeout_atomic(zl3073x_ptp_tod_sem, dpll,
					val, !(DPLL_TOD_CTRL_SEM & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		return ret;

	/* Read the second and nanoseconds */
	zl3073x_read(zl3073x, DPLL_TOD_SEC(dpll->index),
		     sec, DPLL_TOD_SEC_SIZE);
	zl3073x_read(zl3073x, DPLL_TOD_NSEC(dpll->index),
		     nsec, DPLL_TOD_NSEC_SIZE);

	zl3073x_ptp_bytearray_to_timestamp(ts, sec, nsec);

	return 0;
}

static int zl3073x_ptp_gettime64(struct ptp_clock_info *ptp,
				 struct timespec64 *ts)
{
	struct zl3073x_dpll *dpll = container_of(ptp, struct zl3073x_dpll, info);
	struct zl3073x *zl3073x = dpll->zl3073x;
	int ret;

	mutex_lock(zl3073x->lock);
	ret = _zl3073x_ptp_gettime64(dpll, ts, ZL3073X_TOD_CTRL_CMD_READ);
	mutex_unlock(zl3073x->lock);

	return ret;
}

static int _zl3073x_ptp_settime64(struct zl3073x_dpll *dpll,
				  const struct timespec64 *ts,
				  enum zl3073x_tod_ctrl_cmd_t cmd)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	u8 nsec[DPLL_TOD_NSEC_SIZE];
	u8 sec[DPLL_TOD_SEC_SIZE];
	int ret;
	int val;
	u8 ctrl;

	/* Check that the semaphore is clear */
	ret = readx_poll_timeout_atomic(zl3073x_ptp_tod_sem, dpll,
					val, !(DPLL_TOD_CTRL_SEM & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		return ret;

	/* Convert input to something that DPLL can understand */
	zl3073x_ptp_timestamp_to_bytearray(ts, sec, nsec);

	/* Write the value */
	zl3073x_write(zl3073x, DPLL_TOD_SEC(dpll->index),
		      sec, DPLL_TOD_SEC_SIZE);
	zl3073x_write(zl3073x, DPLL_TOD_NSEC(dpll->index),
		      nsec, DPLL_TOD_NSEC_SIZE);

	/* Issue the write command */
	ctrl = DPLL_TOD_CTRL_SEM | cmd;
	zl3073x_write(zl3073x, DPLL_TOD_CTRL(dpll->index), &ctrl, sizeof(ctrl));

	return 0;
}

static int zl3073x_ptp_settime64(struct ptp_clock_info *ptp,
				 const struct timespec64 *ts)
{
	struct zl3073x_dpll *dpll = container_of(ptp, struct zl3073x_dpll, info);
	struct zl3073x *zl3073x = dpll->zl3073x;
	int ret;

	mutex_lock(zl3073x->lock);
	ret = _zl3073x_ptp_settime64(dpll, ts, ZL3073X_TOD_CTRL_CMD_WRITE_NEXT_1HZ);
	mutex_unlock(zl3073x->lock);

	return ret;
}

static int zl3073x_ptp_wait_sec_rollover(struct zl3073x_dpll *dpll)
{
	struct timespec64 init_ts, ts;
	int val;
	int ret;

	memset(&init_ts, 0, sizeof(init_ts));

	do {
		/* Check that the semaphore is clear */
		ret = readx_poll_timeout_atomic(zl3073x_ptp_tod_sem, dpll,
						val, !(DPLL_TOD_CTRL_SEM & val),
						READ_SLEEP_US, READ_TIMEOUT_US);
		if (ret)
			return ret;

		/* Read the time */
		ret = _zl3073x_ptp_gettime64(dpll, &ts,
					     ZL3073X_TOD_CTRL_CMD_READ_NEXT_1HZ);
		if (ret)
			return ret;

		/* Determin if the second has roll over */
		if (!init_ts.tv_sec) {
			init_ts = ts;
		} else {
			if (init_ts.tv_sec < ts.tv_sec)
				break;
		}

		msleep(10);
	} while (true);

	return 0;
}

static s64 _zl3073x_ptp_get_synth_freq(struct zl3073x_dpll *dpll, u8 synth)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	u16 numerator;
	u16 denomitor;
	u8 buf[4];
	u16 base;
	u32 mult;
	int ret;
	int val;

	/* Select the synth */
	memset(buf, 0, sizeof(buf));
	buf[0] = BIT(synth);
	zl3073x_write(zl3073x, DPLL_SYNTH_MB_MASK, buf,
		      DPLL_SYNTH_MB_MASK_SIZE);

	/* Select read command */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_SYNTH_MB_SEM_RD;
	zl3073x_write(zl3073x, DPLL_SYNTH_MB_SEM, buf,
		      DPLL_SYNTH_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_ptp_synth_mb_sem, dpll,
					val,
					!(DPLL_SYNTH_MB_SEM_RD & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		return ret;

	/* The output frequency is determined by the following formula:
	 * base * multiplier * numerator / denomitor
	 * Therefore get all this number and calculate the output frequency
	 */
	zl3073x_read(zl3073x, DPLL_SYNTH_FREQ_BASE, buf,
		     DPLL_SYNTH_FREQ_BASE_SIZE);
	base = buf[0] << 8;
	base |= buf[1];

	zl3073x_read(zl3073x, DPLL_SYNTH_FREQ_MULT, buf,
		     DPLL_SYNTH_FREQ_MULT_SIZE);
	mult = buf[0] << 24;
	mult |= buf[1] << 16;
	mult |= buf[2] << 8;
	mult |= buf[3];

	zl3073x_read(zl3073x, DPLL_SYNTH_FREQ_M, buf,
		     DPLL_SYNTH_FREQ_M_SIZE);
	numerator = buf[0] << 8;
	numerator |= buf[1];

	zl3073x_read(zl3073x, DPLL_SYNTH_FREQ_N, buf,
		     DPLL_SYNTH_FREQ_N_SIZE);
	denomitor = buf[0] << 8;
	denomitor |= buf[1];

	return base * mult * numerator / denomitor;
}

static int zl3073x_ptp_getmaxphase(struct ptp_clock_info *ptp)
{
	/* Adjphase accepts phase inputs from -1s to 1s */
	return NSEC_PER_SEC;
}

static int zl3073x_ptp_adjphase(struct ptp_clock_info *ptp, s32 delta)
{
	struct zl3073x_dpll *dpll = container_of(ptp, struct zl3073x_dpll, info);
	struct zl3073x *zl3073x = dpll->zl3073x;
	u8 tieWriteOp = DPLL_TIE_CTRL_OPERATION;
	s64 delta_sub_sec_in_tie_units;
	u8 tieDpll = BIT(dpll->index);
	s32 delta_sub_sec_in_ns;
	u8 tieData[6];
	int val;
	int ret;

	/* Remove seconds and convert to 0.01ps units */
	delta_sub_sec_in_ns = delta % NSEC_PER_SEC;
	delta_sub_sec_in_tie_units =  delta_sub_sec_in_ns * 100000LL;

	tieData[5] = (delta_sub_sec_in_tie_units & 0xFF0000000000) >> 40;
	tieData[4] = (delta_sub_sec_in_tie_units & 0xFF00000000) >> 32;
	tieData[3] = (delta_sub_sec_in_tie_units & 0xFF000000) >> 24;
	tieData[2] = (delta_sub_sec_in_tie_units & 0xFF0000) >> 16;
	tieData[1] = (delta_sub_sec_in_tie_units & 0xFF00) >> 8;
	tieData[0] = (delta_sub_sec_in_tie_units & 0xFF) >> 0;

	mutex_lock(zl3073x->lock);

	/* Set the ctrl to look at the correct dpll */
	zl3073x_write(zl3073x, DPLL_TIE_CTRL_MASK_REG, &tieDpll, DPLL_TIE_CTRL_SIZE);

	/* Wait for access to the CTRL register */
	ret = readx_poll_timeout_atomic(zl3073x_ptp_tie_ctrl_op, dpll,
					val,
					!(DPLL_TIE_CTRL_MASK & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto out;

	/* Writes data to the tie register */
	zl3073x_write(zl3073x, DPLL_TIE_DATA(dpll->index), tieData, sizeof(tieData));

	/* Request to write the TIE */
	zl3073x_write(zl3073x, DPLL_TIE_CTRL, &tieWriteOp, DPLL_TIE_CTRL_SIZE);

	/* Wait until the TIE operation is completed*/
	ret = readx_poll_timeout_atomic(zl3073x_ptp_tie_ctrl_op, dpll,
					val,
					!(DPLL_TIE_CTRL_MASK & val),
					READ_SLEEP_US, READ_TIMEOUT_US);

out:
	mutex_unlock(zl3073x->lock);

	return ret;
}

static int _zl3073x_ptp_steptime(struct zl3073x_dpll *dpll, const s64 delta)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	s32 register_units;
	u8 buf[4];
	u8 synth;
	int val;
	int ret;

	/* Wait for the previous command to finish */
	ret = readx_poll_timeout_atomic(zl3073x_ptp_phase_ctrl_op, dpll,
					val,
					!(DPLL_OUTPUT_PHASE_STEP_CTRL_OP_MASK & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		return ret;

	/* Set the number of steps to take, the value is 1 as we want to finish
	 * fast
	 */
	memset(buf, 0, sizeof(buf));
	buf[0] = 1;
	zl3073x_write(zl3073x, DPLL_OUTPUT_PHASE_STEP_NUMBER, buf,
		      DPLL_OUTPUT_PHASE_STEP_NUMBER_SIZE);

	/* Get the synth that is connected to the output, it is OK to get the
	 * synth for only 1 output as it is expected that all the outputs that
	 * are used by 1PPS are connected to same synth.
	 */
	zl3073x_read(zl3073x, DPLL_OUTPUT_CTRL(__ffs(dpll->perout_mask)), buf,
		     DPLL_OUTPUT_CTRL_SIZE);
	synth = DPLL_OUTPUT_CTRL_SYNTH_SEL_GET(buf[0]);

	/* Configure the step */
	register_units = div_s64(delta * _zl3073x_ptp_get_synth_freq(dpll, synth),
				 NSEC_PER_SEC);

	memset(buf, 0, sizeof(buf));
	buf[0] = register_units & 0xff;
	buf[1] = (register_units >> 8) & 0xff;
	buf[2] = (register_units >> 16) & 0xff;
	buf[3] = (register_units >> 24) & 0xff;
	zl3073x_write(zl3073x, DPLL_OUTPUT_PHASE_STEP_DATA, buf,
		      DPLL_OUTPUT_PHASE_STEP_DATA_SIZE);

	/* Select which output should be adjusted */
	memset(buf, 0, sizeof(buf));
	buf[0] = dpll->perout_mask;
	zl3073x_write(zl3073x, DPLL_OUTPUT_PHASE_STEP_MASK, buf,
		      DPLL_OUTPUT_PHASE_STEP_MASK_SIZE);

	/* Start the phase adjustment on the output pin and also on the ToD */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_OUTPUT_PHASE_STEP_CTRL_DPLL(dpll->index) |
		 DPLL_OUTPUT_PHASE_STEP_CTRL_OP(DPLL_OUTPUT_PAHSE_STEP_CTRL_OP_WRITE) |
		 DPLL_OUTPUT_PHASE_STEP_CTRL_TOD_STEP;

	zl3073x_write(zl3073x, DPLL_OUTPUT_PHASE_STEP_CTRL, buf,
		      DPLL_OUTPUT_PHASE_STEP_CTRL_SIZE);

	return 0;
}

static int zl3073x_dpll_raw_mode_get(struct zl3073x *zl3073x, int dpll_index)
{
	u8 mode;

	zl3073x_read(zl3073x, DPLL_MODE_REFSEL(dpll_index), &mode, sizeof(mode));
	return DPLL_MODE_REFSEL_MODE_GET(mode);
}

static int zl3073x_dpll_map_raw_to_manager_mode(int raw_mode)
{
	switch (raw_mode)	{
	case ZL3073X_MODE_HOLDOVER:
	case ZL3073X_MODE_REFLOCK:
		return DPLL_MODE_MANUAL;
	case ZL3073X_MODE_AUTO_LOCK:
		return DPLL_MODE_AUTOMATIC;
	case ZL3073X_MODE_FREERUN:
	case ZL3073X_MODE_NCO:
	default:
		return -EINVAL;
	}
}

static int zl3073x_dpll_raw_lock_status_get(struct zl3073x *zl3073x, int dpll_index)
{
	u8 dpll_status;

	zl3073x_read(zl3073x, DPLL_LOCK_REFSEL_STATUS(dpll_index), &dpll_status, sizeof(dpll_status));
	return DPLL_LOCK_REFSEL_LOCK_GET(dpll_status);
}

static int zl3073x_dpll_map_raw_to_manager_lock_status(struct zl3073x *zl3073x,
				int dpll_index, u8 dpll_status)
{
	u8 dpll_mon_status;
	u8 ho_ready;

	zl3073x_read(zl3073x, DPLL_MON_STATUS(dpll_index), &dpll_mon_status, sizeof(dpll_mon_status));
	ho_ready = DPLL_MON_STATUS_HO_READY_GET(dpll_mon_status);

	switch (dpll_status)	{
	case ZLS3073X_DPLL_STATE_FREERUN:
	case ZLS3073X_DPLL_STATE_FAST_LOCK:
	case ZLS3073X_DPLL_STATE_ACQUIRING:
		return DPLL_LOCK_STATUS_UNLOCKED;
	case ZLS3073X_DPLL_STATE_HOLDOVER:
		return DPLL_LOCK_STATUS_HOLDOVER;
	case ZLS3073X_DPLL_STATE_LOCK:
		if (ho_ready)
			return DPLL_LOCK_STATUS_LOCKED_HO_ACQ;
		else
			return DPLL_LOCK_STATUS_LOCKED;

	default:
		return -EINVAL;
	}
}

static int zl3073x_dpll_get_priority_ref(struct zl3073x *zl3073x, u8 dpll_index,
				u8 refId, u32 *prio)
{
	u8 ref_priority;
	u8 buf[3];
	int ret;
	int val;

	mutex_lock(zl3073x->lock);

	memset(buf, 0, sizeof(buf));
	buf[0] = BIT(dpll_index);
	zl3073x_write(zl3073x, DPLL_DPLL_MB_MASK, buf, DPLL_DPLL_MB_MASK_SIZE);

	/* Select read command */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_DPLL_MB_SEM_RD;
	zl3073x_write(zl3073x, DPLL_DPLL_MB_SEM, buf, DPLL_DPLL_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_dpll_mb_sem, zl3073x, val,
					!(DPLL_DPLL_MB_SEM_RD & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto out;

	zl3073x_read(zl3073x, DPLL_REF_PRIORITY(refId), &ref_priority, sizeof(ref_priority));
	*prio = DPLL_REF_PRIORITY_GET(ref_priority, refId);

out:
	mutex_unlock(zl3073x->lock);
	return ret;
}


static int zl3073x_dpll_set_priority_ref(struct zl3073x *zl3073x, u8 dpll_index,
				u8 refId, u32 new_priority)
{
	u8 current_priority;
	u8 updated_priority;
	u8 buf[3];
	int ret;
	int val;

	ret = 0;

	mutex_lock(zl3073x->lock);

	memset(buf, 0, sizeof(buf));
	buf[0] = BIT(dpll_index);
	zl3073x_write(zl3073x, DPLL_DPLL_MB_MASK, buf, DPLL_DPLL_MB_MASK_SIZE);

	/* Select read command */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_DPLL_MB_SEM_RD;
	zl3073x_write(zl3073x, DPLL_DPLL_MB_SEM, buf, DPLL_DPLL_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_dpll_mb_sem, zl3073x, val,
					!(DPLL_DPLL_MB_SEM_RD & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto out;

    /*	Read the current priority to preserve the other nibble	*/
	zl3073x_read(zl3073x, DPLL_REF_PRIORITY(refId), &current_priority, sizeof(current_priority));

    /*	Update the priority using the macro	*/
	updated_priority = DPLL_REF_PRIORITY_SET(current_priority, refId, new_priority);

	/*	Write the updated priority value	*/
	buf[0] = updated_priority;
	zl3073x_write(zl3073x, DPLL_REF_PRIORITY(refId), &buf[0], sizeof(buf[0]));

    /* Select write command */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_DPLL_MB_SEM_WR;
	zl3073x_write(zl3073x, DPLL_DPLL_MB_SEM, buf, DPLL_DPLL_MB_SEM_SIZE);

    /* Wait for the write command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_dpll_mb_sem, zl3073x, val,
										!(DPLL_DPLL_MB_SEM_WR & val),
										READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto out;

out:
	mutex_unlock(zl3073x->lock);

	return ret;
}


static int zl3073x_dpll_get_input_phase_adjust(struct zl3073x *zl3073x, u8 refId, s32 *phaseAdj)
{
	u8 buf[DPLL_REF_PHASE_OFFSET_COMPENSATION_REG_SIZE];
	s64 currentPhaseOffsetComp = 0;
	s32 phaseOffsetComp32 = 0;
	int ret;
	int val;

	mutex_lock(zl3073x->lock);

	memset(buf, 0, sizeof(buf));
	buf[0] = BIT(refId);
	zl3073x_write(zl3073x, DPLL_REF_MB_MASK, buf, DPLL_DPLL_MB_MASK_SIZE);

	/* Select read command */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_REF_MB_SEM_RD;
	zl3073x_write(zl3073x, DPLL_REF_MB_SEM, buf, DPLL_REF_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_dpll_mb_sem, zl3073x, val,
					!(DPLL_REF_MB_SEM_RD & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto out;

	memset(buf, 0, sizeof(buf));
	zl3073x_read(zl3073x, DPLL_REF_PHASE_OFFSET_COMPENSATION_REG, buf, DPLL_REF_PHASE_OFFSET_COMPENSATION_REG_SIZE);

	/* Combine the 6 bytes into a 64-bit signed integer */
	currentPhaseOffsetComp = ((s64)buf[0] << 40) |
								((s64)buf[1] << 32) |
								((s64)buf[2] << 24) |
								((s64)buf[3] << 16) |
								((s64)buf[4] << 8) |
								((s64)buf[5] << 0);

	/* Perform sign extension for a 48-bit signed value */
	if (currentPhaseOffsetComp & (1LL << 47))
		currentPhaseOffsetComp |= 0xFFFF000000000000;

	/* Check if the value fits within Sint32 range */
	if (currentPhaseOffsetComp < phase_range.min || currentPhaseOffsetComp > phase_range.max) {
		ret = -ERANGE; /* Value too large to fit in Sint32 */
		goto out;
	} else {
		/* Convert to 32-bit signed integer */
		phaseOffsetComp32 = (s32)currentPhaseOffsetComp;
		/* Reverse the two's complement negation applied during 'set' */
		*phaseAdj = ~phaseOffsetComp32 + 1;
	}

out:
	mutex_unlock(zl3073x->lock);
	return ret;
}

static int zl3073x_dpll_set_input_phase_adjust(struct zl3073x *zl3073x, u8 refId, s32 phaseOffsetComp32)
{
	u8 buf[DPLL_REF_PHASE_OFFSET_COMPENSATION_REG_SIZE];
	s64 phaseOffsetComp48 = 0;
	int ret;
	int val;

	/* Convert the 32-bit signed value to 64-bit format */
	phaseOffsetComp48 = (s64)phaseOffsetComp32;

    /* Mask the upper 16 bits to ensure it's within 48 bits */
	phaseOffsetComp48 &= 0xFFFFFFFFFFFF;

	mutex_lock(zl3073x->lock);

	memset(buf, 0, sizeof(buf));
	buf[0] = BIT(refId);
	zl3073x_write(zl3073x, DPLL_REF_MB_MASK, buf, DPLL_DPLL_MB_MASK_SIZE);

	memset(buf, 0, sizeof(buf));
	/* 2's compliment */
	phaseOffsetComp48 = ~phaseOffsetComp48 + 1;

	/* Split the 48-bit value into 6 bytes */
	buf[5] = (u8)(phaseOffsetComp48 >> 40);
	buf[4] = (u8)(phaseOffsetComp48 >> 32);
	buf[3] = (u8)(phaseOffsetComp48 >> 24);
	buf[2] = (u8)(phaseOffsetComp48 >> 16);
	buf[1] = (u8)(phaseOffsetComp48 >> 8);
	buf[0] = (u8)(phaseOffsetComp48 >> 0);

	/* Write the 48-bit value to the compensation register */
	zl3073x_write(zl3073x, DPLL_REF_PHASE_OFFSET_COMPENSATION_REG, buf, DPLL_REF_PHASE_OFFSET_COMPENSATION_REG_SIZE);

	/* Select write command */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_REF_MB_SEM_WR;
	zl3073x_write(zl3073x, DPLL_REF_MB_SEM, buf, DPLL_REF_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_dpll_mb_sem, zl3073x, val,
					!(DPLL_REF_MB_SEM_WR & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto out;

out:
	mutex_unlock(zl3073x->lock);
	return ret;
}

static int zl3073x_dpll_get_output_phase_adjust(struct zl3073x *zl3073x, u8 outputIndex, s32 *phaseAdj)
{
	u8 buf[DPLL_OUTPUT_PHASE_COMPENSATION_REG_SIZE];
	const u64 scaleSecToPicoSec = 1000000000000;
	s32 currentPhaseOffsetComp = 0;
	int halfSynthCycle;
	u8 synth;
	u32 freq;
	int ret;
	int val;

	synth = zl3073x_synth_get(zl3073x, outputIndex);
	freq = _zl3073x_ptp_get_synth_freq(zl3073x->dpll, synth);
	halfSynthCycle = div_u64(scaleSecToPicoSec, (freq*2));

	mutex_lock(zl3073x->lock);

	memset(buf, 0, sizeof(buf));
	buf[0] = BIT(outputIndex/2);
	zl3073x_write(zl3073x, DPLL_OUTPUT_MB_MASK, buf, DPLL_OUTPUT_MB_MASK_SIZE);

	/* Select read command */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_OUTPUT_MB_SEM_RD;
	zl3073x_write(zl3073x, DPLL_OUTPUT_MB_SEM, buf, DPLL_OUTPUT_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_dpll_mb_sem, zl3073x, val,
					!(DPLL_OUTPUT_MB_SEM_RD & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto out;

	memset(buf, 0, sizeof(buf));
	zl3073x_read(zl3073x, DPLL_OUTPUT_PHASE_COMPENSATION_REG, buf, DPLL_OUTPUT_PHASE_COMPENSATION_REG_SIZE);

	/* Combine the 4 bytes into a 32-bit signed integer */
	currentPhaseOffsetComp = (buf[0] << 24) |
								(buf[1] << 16) |
								(buf[2] << 8) |
								(buf[3] << 0);

	if (currentPhaseOffsetComp != 0) {
		currentPhaseOffsetComp = (currentPhaseOffsetComp * halfSynthCycle);
		*phaseAdj = ~currentPhaseOffsetComp + 1; /* Reverse the two's complement negation applied during 'set' */
	}

out:
	mutex_unlock(zl3073x->lock);
	return ret;
}

static int zl3073x_dpll_set_output_phase_adjust(struct zl3073x *zl3073x, u8 outputIndex, s32 phaseOffsetComp32)
{
	u8 buf[DPLL_OUTPUT_PHASE_COMPENSATION_REG_SIZE];
	const u64 scaleSecToPicoSec = 1000000000000;
	int halfSynthCycle;
	u8 synth;
	u32 freq;
	int ret;
	int val;

	synth = zl3073x_synth_get(zl3073x, outputIndex);
	freq = _zl3073x_ptp_get_synth_freq(zl3073x->dpll, synth);
	halfSynthCycle = div_u64(scaleSecToPicoSec, (freq*2));

	if ((halfSynthCycle % phaseOffsetComp32) != 0) {
		/* Not a multiple of halfSynthCycle, return an error */
		ret = -ERANGE;
		goto out;
	}

	mutex_lock(zl3073x->lock);

	memset(buf, 0, sizeof(buf));
	buf[0] = BIT(outputIndex/2);
	zl3073x_write(zl3073x, DPLL_OUTPUT_MB_MASK, buf, DPLL_OUTPUT_MB_MASK_SIZE);

	memset(buf, 0, sizeof(buf));

	phaseOffsetComp32 = phaseOffsetComp32 / halfSynthCycle;
	/* 2's compliment */
	phaseOffsetComp32 = ~phaseOffsetComp32 + 1;

	/* Split the 32-bit value into 4 bytes */
	buf[3] = (u8)(phaseOffsetComp32 >> 24);
	buf[2] = (u8)(phaseOffsetComp32 >> 16);
	buf[1] = (u8)(phaseOffsetComp32 >> 8);
	buf[0] = (u8)(phaseOffsetComp32 >> 0);

	/* Write the 48-bit value to the compensation register */
	zl3073x_write(zl3073x, DPLL_OUTPUT_PHASE_COMPENSATION_REG, buf, DPLL_OUTPUT_PHASE_COMPENSATION_REG_SIZE);

	/* Select write command */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_REF_MB_SEM_WR;
	zl3073x_write(zl3073x, DPLL_OUTPUT_MB_SEM, buf, DPLL_OUTPUT_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_dpll_mb_sem, zl3073x, val,
					!(DPLL_OUTPUT_MB_SEM_WR & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto out;

out:
	mutex_unlock(zl3073x->lock);
	return 0;
}

static int zl3073x_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct zl3073x_dpll *dpll = container_of(ptp, struct zl3073x_dpll, info);
	struct zl3073x *zl3073x = dpll->zl3073x;
	s64 delta_sub_sec_in_ns;
	struct timespec64 ts;
	s64 delta_sec_in_ns;
	s32 delta_sec_rem;
	s64 delta_sec;
	int ret;
	int val;

	/* Split the offset to apply into seconds and nanoseconds */
	delta_sec = div_s64_rem(delta, NSEC_PER_SEC, &delta_sec_rem);
	delta_sec_in_ns = delta_sec * NSEC_PER_SEC;
	delta_sub_sec_in_ns = delta_sec_rem;

	mutex_lock(zl3073x->lock);

	if (delta >= NSEC_PER_SEC || delta <= -NSEC_PER_SEC) {
		/* wait for rollover */
		ret = zl3073x_ptp_wait_sec_rollover(dpll);
		if (ret)
			goto out;

		/* get the predicted TOD at the next internal 1PPS */
		ret = _zl3073x_ptp_gettime64(dpll, &ts,
					     ZL3073X_TOD_CTRL_CMD_READ_NEXT_1HZ);
		if (ret)
			goto out;

		ts = timespec64_add(ts, ns_to_timespec64(delta_sec_in_ns));

		ret = _zl3073x_ptp_settime64(dpll, &ts,
					     ZL3073X_TOD_CTRL_CMD_WRITE_NEXT_1HZ);
		if (ret)
			goto out;

		/* Wait for the semaphore bit to confirm correct settime application */
		ret = readx_poll_timeout_atomic(zl3073x_ptp_tod_sem, dpll,
					val, !(DPLL_TOD_CTRL_SEM & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
		if (ret)
			goto out;
	}

	ret = _zl3073x_ptp_steptime(dpll, delta_sub_sec_in_ns);

out:
	mutex_unlock(zl3073x->lock);

	return ret;
}

static int zl3073x_ptp_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct zl3073x_dpll *dpll = container_of(ptp, struct zl3073x_dpll, info);
	struct zl3073x *zl3073x = dpll->zl3073x;
	s64 scaled_ppm_s64;
	u8 dco[6];
	s64 ref;

	/* Store the scaled_ppm into a s64 variable because on 32bit arch, the
	 * multiplication with ZL30373X_1PPM_FORMAT with overflow meaning that
	 * will not be able to adjust to lowest ns
	 */
	scaled_ppm_s64 = scaled_ppm;
	if (!scaled_ppm_s64)
		return 0;

	mutex_lock(zl3073x->lock);

	ref = ZL3073X_1PPM_FORMAT * (scaled_ppm_s64 >> 16);
	ref += (ZL3073X_1PPM_FORMAT * (0xffff & scaled_ppm_s64)) >> 16;

	/* The value that is written in HW is in 2 complement */
	ref = ~ref + 1;

	dco[5] = ref >> 40;
	dco[4] = ref >> 32;
	dco[3] = ref >> 24;
	dco[2] = ref >> 16;
	dco[1] = ref >>  8;
	dco[0] = ref >>  0;

	zl3073x_write(zl3073x, DPLL_DF_OFFSET(dpll->index), dco, sizeof(dco));

	mutex_unlock(zl3073x->lock);

	return 0;
}

static enum zl3073x_output_mode_signal_format_t
_zl3073x_ptp_disable_pin(enum zl3073x_output_mode_signal_format_t current_mode,
			 u8 pin)
{
	switch (current_mode) {
	case ZL3073X_P_ENABLE:
		if (ZL3073X_P_PIN(pin))
			return ZL3073X_BOTH_DISABLED;
		break;
	case ZL3073X_N_ENABLE:
		if (ZL3073X_N_PIN(pin))
			return ZL3073X_BOTH_DISABLED;
		break;
	case ZL3073X_BOTH_ENABLED:
		if (ZL3073X_P_PIN(pin))
			return ZL3073X_N_ENABLE;
		else
			return ZL3073X_P_ENABLE;
	default:
		return ZL3073X_BOTH_DISABLED;
	}

	return ZL3073X_BOTH_DISABLED;
}

static int zl3073x_ptp_perout_disable(struct zl3073x_dpll *dpll,
				      struct ptp_perout_request *perout)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	u8 buf[2];
	int pin;
	int ret;
	int val;
	u8 mode;

	pin = ptp_find_pin(dpll->clock, PTP_PF_PEROUT, perout->index);
	if (pin == -1 || pin >= ZL3073X_MAX_OUTPUT_PINS)
		return -EINVAL;

	/* Select the output pin */
	memset(buf, 0, sizeof(buf));
	buf[0] = BIT(pin / 2);
	zl3073x_write(zl3073x, DPLL_OUTPUT_MB_MASK, buf,
		      DPLL_OUTPUT_MB_MASK_SIZE);

	/* Select read command  */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_OUTPUT_MB_SEM_RD;
	zl3073x_write(zl3073x, DPLL_OUTPUT_MB_SEM, buf,
		      DPLL_OUTPUT_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_ptp_output_mb_sem, dpll,
					val,
					!(DPLL_OUTPUT_MB_SEM_RD & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		return ret;

	/* Read current configuration */
	zl3073x_read(zl3073x, DPLL_OUTPUT_MODE, buf,
		     DPLL_OUTPUT_MODE_SIZE);

	mode = DPLL_OUTPUT_MODE_SIGNAL_FORMAT_GET(buf[0]);
	buf[0] &= ~DPLL_OUTPUT_MODE_SIGNAL_FORMAT_MASK;
	buf[0] |= DPLL_OUTPUT_MODE_SIGNAL_FORMAT(_zl3073x_ptp_disable_pin(mode,
									  pin));

	/* Update the configuration */
	zl3073x_write(zl3073x, DPLL_OUTPUT_MODE, buf,
		      DPLL_OUTPUT_MODE_SIZE);

	/* Select write command */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_OUTPUT_MB_SEM_WR;
	zl3073x_write(zl3073x, DPLL_OUTPUT_MB_SEM, buf,
		      DPLL_OUTPUT_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_ptp_output_mb_sem, dpll,
					val,
					!(DPLL_OUTPUT_MB_SEM_WR & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		return ret;

	dpll->perout_mask &= ~BIT(pin / 2);

	return 0;
}

static enum zl3073x_output_mode_signal_format_t
_zl3073x_ptp_enable_pin(enum zl3073x_output_mode_signal_format_t current_mode,
			u8 pin)
{
	switch (current_mode) {
	case ZL3073X_P_ENABLE:
		if (ZL3073X_N_PIN(pin))
			return ZL3073X_BOTH_ENABLED;
		break;
	case ZL3073X_N_ENABLE:
		if (ZL3073X_P_PIN(pin))
			return ZL3073X_BOTH_ENABLED;
		break;
	case ZL3073X_BOTH_DISABLED:
		if (ZL3073X_P_PIN(pin))
			return ZL3073X_P_ENABLE;
		else
			return ZL3073X_N_ENABLE;
	default:
		return ZL3073X_BOTH_ENABLED;
	}

	return ZL3073X_BOTH_ENABLED;
}

static int zl3073x_ptp_perout_enable(struct zl3073x_dpll *dpll,
				     struct ptp_perout_request *perout)
{
	struct zl3073x *zl3073x = dpll->zl3073x;
	u8 buf[4];
	u32 width;
	u32 freq;
	u8 synth;
	int pin;
	int ret;
	int val;
	u8 mode;

	pin = ptp_find_pin(dpll->clock, PTP_PF_PEROUT, perout->index);
	if (pin == -1 || pin >= ZL3073X_MAX_OUTPUT_PINS)
		return -EINVAL;

	/* Select the output pin */
	memset(buf, 0, sizeof(buf));
	buf[0] = BIT(pin / 2);
	zl3073x_write(zl3073x, DPLL_OUTPUT_MB_MASK, buf,
		      DPLL_OUTPUT_MB_MASK_SIZE);

	/* Select read command  */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_OUTPUT_MB_SEM_RD;
	zl3073x_write(zl3073x, DPLL_OUTPUT_MB_SEM, buf,
		      DPLL_OUTPUT_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_ptp_output_mb_sem, dpll,
					val,
					!(DPLL_OUTPUT_MB_SEM_RD & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		return ret;

	/* Read configuration of output mode */
	zl3073x_read(zl3073x, DPLL_OUTPUT_MODE, buf,
		     DPLL_OUTPUT_MODE_SIZE);

	mode = DPLL_OUTPUT_MODE_SIGNAL_FORMAT_GET(buf[0]);
	buf[0] &= ~DPLL_OUTPUT_MODE_SIGNAL_FORMAT_MASK;
	buf[0] |= DPLL_OUTPUT_MODE_SIGNAL_FORMAT(_zl3073x_ptp_enable_pin(mode,
									 pin));

	/* Update the configuration */
	zl3073x_write(zl3073x, DPLL_OUTPUT_MODE, buf,
		      DPLL_OUTPUT_MODE_SIZE);

	/* Make sure that the output is set as clock and not GPIO */
	buf[0] = 0x0;
	zl3073x_write(zl3073x, DPLL_OUTPUT_GPO_EN, buf, DPLL_OUTPUT_GPO_EN_SIZE);

	/* Get the synth that is connected to the output and set the same value
	 * in the ouput divider of the pin so it can get an 1PPS as this is the
	 * only value supported
	 */
	zl3073x_read(zl3073x, DPLL_OUTPUT_CTRL(pin / 2), buf,
		     DPLL_OUTPUT_CTRL_SIZE);
	synth = DPLL_OUTPUT_CTRL_SYNTH_SEL_GET(buf[0]);
	freq = _zl3073x_ptp_get_synth_freq(dpll, synth);
	memset(buf, 0, sizeof(buf));
	buf[3] = (freq >> 24) & 0xff;
	buf[2] = (freq >> 16) & 0xff;
	buf[1] = (freq >>  8) & 0xff;
	buf[0] = freq & 0xff;
	zl3073x_write(zl3073x, DPLL_OUTPUT_DIV, buf,
		      DPLL_OUTPUT_DIV_SIZE);

	if (perout->flags & PTP_PEROUT_DUTY_CYCLE) {
		if (perout->on.sec)
			return -EINVAL;

		memset(buf, 0, sizeof(buf));

		/* The value that needs to be written in the register is
		 * calculated as following:
		 * width = perout->on.nsec / (NSEC_PER_SEC / freq) * 2
		 * Bellow is just simplify the calculation
		 */
		width = NSEC_PER_SEC / perout->on.nsec;
		width = freq / width;
		width = width * 2;

		buf[3] = (width >> 24) & 0xff;
		buf[2] = (width >> 16) & 0xff;
		buf[1] = (width >>  8) & 0xff;
		buf[0] = width & 0xff;
		zl3073x_write(zl3073x, DPLL_OUTPUT_WIDTH, buf,
			      DPLL_OUTPUT_WIDTH_SIZE);
	}

	/* Select write command */
	memset(buf, 0, sizeof(buf));
	buf[0] = DPLL_OUTPUT_MB_SEM_WR;
	zl3073x_write(zl3073x, DPLL_OUTPUT_MB_SEM, buf,
		      DPLL_OUTPUT_MB_SEM_SIZE);

	/* Wait for the command to actually finish */
	ret = readx_poll_timeout_atomic(zl3073x_ptp_output_mb_sem, dpll,
					val,
					!(DPLL_OUTPUT_MB_SEM_WR & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		return ret;

	dpll->perout_mask |= BIT(pin / 2);

	return 0;
}

static int zl3073x_ptp_enable(struct ptp_clock_info *ptp,
			      struct ptp_clock_request *rq, int on)
{
	struct zl3073x_dpll *dpll = container_of(ptp, struct zl3073x_dpll, info);
	struct zl3073x *zl3073x = dpll->zl3073x;
	int err;

	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
		mutex_lock(zl3073x->lock);
		if (!on)
			err = zl3073x_ptp_perout_disable(dpll, &rq->perout);
		/* Only accept a 1-PPS aligned to the second. */
		else if (rq->perout.start.nsec || rq->perout.period.sec != 1 ||
			 rq->perout.period.nsec)
			err = -ERANGE;
		else
			err = zl3073x_ptp_perout_enable(dpll, &rq->perout);
		mutex_unlock(zl3073x->lock);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return err;
}

static int zl3073x_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin,
			      enum ptp_pin_function func, unsigned int chan)
{
	switch (func) {
	case PTP_PF_NONE:
	case PTP_PF_PEROUT:
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static struct ptp_clock_info zl3073x_ptp_clock_info = {
	.owner		= THIS_MODULE,
	.name		= "zl3073x ptp",
	.max_adj	= 1000000000,
	.gettime64	= zl3073x_ptp_gettime64,
	.settime64	= zl3073x_ptp_settime64,
	.adjtime	= zl3073x_ptp_adjtime,
	.adjfine	= zl3073x_ptp_adjfine,
	.adjphase	= zl3073x_ptp_adjphase,
	.getmaxphase	= zl3073x_ptp_getmaxphase,
	.enable		= zl3073x_ptp_enable,
	.verify		= zl3073x_ptp_verify,
	.n_per_out	= ZL3073X_MAX_OUTPUT_PINS,
	.n_ext_ts	= ZL3073X_MAX_OUTPUT_PINS,
	.n_pins		= ZL3073X_MAX_OUTPUT_PINS,
};

/* DPLL Supporting Funtions */
static int zl3073x_dpll_ref_phase_err_rqst_op(struct zl3073x *zl3073x)
{
	u8 read_rqst;

	zl3073x_read(zl3073x, DPLL_REF_PHASE_ERR_RQST, &read_rqst, sizeof(read_rqst));
	return read_rqst;
}

static int zl3073x_dpll_ref_freq_meas_op(struct zl3073x *zl3073x)
{
	u8 ref_freq_meas_ctrl;

	zl3073x_read(zl3073x, REF_FREQ_MEAS_CTRL, &ref_freq_meas_ctrl, sizeof(ref_freq_meas_ctrl));
	return ref_freq_meas_ctrl;
}

static int zl3073x_dpll_forced_ref_get(struct zl3073x *zl3073x, int dpll_index)
{
	u8 ref;

	zl3073x_read(zl3073x, DPLL_MODE_REFSEL(dpll_index), &ref, sizeof(ref));
	return DPLL_MODE_REFSEL_REF_GET(ref);
}

static int zl3073x_dpll_ref_selected_get(struct zl3073x *zl3073x, int dpll_index)
{
	u8 selected_ref;

	zl3073x_read(zl3073x, DPLL_LOCK_REFSEL_STATUS(dpll_index), &selected_ref, sizeof(selected_ref));
	return DPLL_LOCK_REFSEL_REF_GET(selected_ref);
}

static int zl3073x_dpll_ref_status_get(struct zl3073x *zl3073x, int ref_index)
{
	u8 ref_status;

	zl3073x_read(zl3073x, DPLL_REF_MON_STATUS(ref_index), &ref_status, sizeof(ref_status));
	return ref_status;
}

static int zl3073x_dpll_get(struct zl3073x *zl3073x, int synth)
{
	u8 synth_ctrl;

	zl3073x_read(zl3073x, DPLL_SYNTH_CTRL(synth), &synth_ctrl,
		     sizeof(synth_ctrl));
	return DPLL_SYNTH_CTRL_DPLL_SEL_GET(synth_ctrl);
}

static int zl3073x_dpll_phase_offset_get(struct zl3073x *zl3073x, struct zl3073x_dpll *zl3073x_dpll,
				u8 ref_index, s64 *phase_offset)
{
	s64 phase_offset_reg_units;
	u8 read_rqst = 0b1;
	u8 dpll_meas_ctrl;
	u8 dpll_meas_idx;
	u8 phase_err[6];
	int ret;
	int val;

	dpll_meas_idx = (zl3073x_dpll->index) & DPLL_MEAS_IDX_MASK;

	mutex_lock(zl3073x->lock);

	ret = readx_poll_timeout_atomic(zl3073x_dpll_ref_phase_err_rqst_op, zl3073x,
					val,
					!(DPLL_REF_PHASE_ERR_RQST_MASK & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto err;

	zl3073x_write(zl3073x, DPLL_MEAS_IDX_REG, &dpll_meas_idx, sizeof(dpll_meas_idx));

	zl3073x_read(zl3073x, DPLL_MEAS_CTRL, &dpll_meas_ctrl, sizeof(dpll_meas_ctrl));
	dpll_meas_ctrl |= 0b1;
	zl3073x_write(zl3073x, DPLL_MEAS_CTRL, &dpll_meas_ctrl, sizeof(dpll_meas_ctrl));

	zl3073x_write(zl3073x, DPLL_REF_PHASE_ERR_RQST, &read_rqst, sizeof(read_rqst));

	ret = readx_poll_timeout_atomic(zl3073x_dpll_ref_phase_err_rqst_op, zl3073x,
					val,
					!(DPLL_REF_PHASE_ERR_RQST_MASK & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto err;

	zl3073x_read(zl3073x, DPLL_REF_PHASE_ERR(ref_index), phase_err, sizeof(phase_err));

	mutex_unlock(zl3073x->lock);

	phase_offset_reg_units = 0;
	phase_offset_reg_units |= ((s64)phase_err[5] << 0);
	phase_offset_reg_units |= ((s64)phase_err[4] << 8);
	phase_offset_reg_units |= ((s64)phase_err[3] << 16);
	phase_offset_reg_units |= ((s64)phase_err[2] << 24);
	phase_offset_reg_units |= ((s64)phase_err[1] << 32);
	phase_offset_reg_units |= ((s64)phase_err[0] << 40);


	/* Perform sign extension for a 48-bit signed value */
	if (phase_offset_reg_units & (1LL << 47))
		phase_offset_reg_units |= 0xFFFF000000000000;

	/* The register units are 0.01 ps, and the offset is returned in units of ps. */
	*phase_offset = div_s64(phase_offset_reg_units, 100);

	return ret;

err:
	mutex_unlock(zl3073x->lock);
	*phase_offset = 0;

	return ret;
}


static int zl3073x_dpll_pin_frequency_set(const struct dpll_pin *pin, void *pin_priv,
			     const struct dpll_device *dpll, void *dpll_priv,
			     const u64 frequency,
			     struct netlink_ext_ack *extack)
{
	return -EOPNOTSUPP;
}

static int zl3073x_dpll_pin_frequency_get(const struct dpll_pin *pin, void *pin_priv,
			     const struct dpll_device *dpll, void *dpll_priv,
			     u64 *frequency, struct netlink_ext_ack *extack)
{
	return -EOPNOTSUPP;
}

static int zl3073x_dpll_pin_direction_get(const struct dpll_pin *pin, void *pin_priv,
			     const struct dpll_device *dpll, void *dpll_priv,
			     enum dpll_pin_direction *direction,
			     struct netlink_ext_ack *extack)
{
	struct zl3073x_pin *zl3073x_pin = pin_priv;
	enum dpll_pin_direction pin_direction;

	if (ZL3073X_IS_INPUT_PIN(zl3073x_pin->index))
		pin_direction = DPLL_PIN_DIRECTION_INPUT;
	else
		pin_direction = DPLL_PIN_DIRECTION_OUTPUT;

	*direction = pin_direction;

	return 0;
}

static int zl3073x_input_pin_state_get(struct zl3073x *zl3073x, int dpll_index,
					int ref_index, enum dpll_pin_state *state)
{
	int selected_ref_index;
	int forced_ref_index;
	int ref_priority;
	int ref_status;
	int ret = 0;
	int mode;

	ref_status = zl3073x_dpll_ref_status_get(zl3073x, ref_index);

	if (!DPLL_REF_MON_STATUS_QUALIFIED(ref_status)) {
		*state = DPLL_PIN_STATE_DISCONNECTED;
		goto out;
	}

	mode = zl3073x_dpll_raw_mode_get(zl3073x, dpll_index);
	forced_ref_index = zl3073x_dpll_forced_ref_get(zl3073x, dpll_index);

	if (mode == ZL3073X_MODE_AUTO_LOCK) {
		selected_ref_index = zl3073x_dpll_ref_selected_get(zl3073x, dpll_index);
		ret = zl3073x_dpll_get_priority_ref(zl3073x, dpll_index, ref_index, &ref_priority);

		if (ret)
			goto out;

		if (ref_index == selected_ref_index)
			*state = DPLL_PIN_STATE_CONNECTED;
		else if (ref_priority != DPLL_REF_PRIORITY_INVALID)
			*state = DPLL_PIN_STATE_SELECTABLE;
		else
			*state = DPLL_PIN_STATE_DISCONNECTED;
	} else if (ref_index == forced_ref_index)  {

		*state = DPLL_PIN_STATE_CONNECTED;
	} else {
		*state = DPLL_PIN_STATE_DISCONNECTED;
	}

out:
	return ret;
}

static int zl3073x_output_pin_state_get(struct zl3073x *zl3073x, int dpll_index,
				int output_index, enum dpll_pin_state *state)
{
	int synth_dpll;
	int synth;

	*state = DPLL_PIN_STATE_DISCONNECTED;


	synth = zl3073x_synth_get(zl3073x, (output_index / 2));

	if (ZL3073X_CHECK_SYNTH_ID(synth)) {
		synth_dpll = zl3073x_dpll_get(zl3073x, synth);

		if (synth_dpll == dpll_index)
			*state = DPLL_PIN_STATE_CONNECTED;
	}

	return 0;
}

static int zl3073x_dpll_pin_state_on_dpll_get(const struct dpll_pin *pin, void *pin_priv,
				 const struct dpll_device *dpll,
				 void *dpll_priv, enum dpll_pin_state *state,
				 struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zl3073x_dpll = dpll_priv;
	struct zl3073x_pin *zl3073x_pin = pin_priv;
	int pin_register_index;
	int ret;

	if (ZL3073X_IS_INPUT_PIN(zl3073x_pin->index)) {
		pin_register_index = ZL3073X_REG_MAP_INPUT_PIN_GET(zl3073x_pin->index);
		ret = zl3073x_input_pin_state_get(zl3073x_dpll->zl3073x, zl3073x_dpll->index,
						pin_register_index, state);
	} else {
		ret = zl3073x_output_pin_state_get(zl3073x_dpll->zl3073x, zl3073x_dpll->index,
						zl3073x_pin->index, state);
	}

	return ret;
}

static int zl3073x_dpll_pin_prio_get(const struct dpll_pin *pin,  void *pin_priv,
			const struct dpll_device *dpll,  void *dpll_priv,
			u32 *prio, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zl3073x_dpll = dpll_priv;
	struct zl3073x_pin *zl3073x_pin = pin_priv;
	int pin_register_index;
	int ret;

	if (ZL3073X_IS_INPUT_PIN(zl3073x_pin->index)) {
		pin_register_index = ZL3073X_REG_MAP_INPUT_PIN_GET(zl3073x_pin->index);
		ret = zl3073x_dpll_get_priority_ref(zl3073x_dpll->zl3073x, zl3073x_dpll->index,
						pin_register_index, prio);
	} else {
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static int zl3073x_dpll_pin_prio_set(const struct dpll_pin *pin, void *pin_priv,
			const struct dpll_device *dpll, void *dpll_priv,
			const u32 prio, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zl3073x_dpll = dpll_priv;
	struct zl3073x_pin *zl3073x_pin = pin_priv;
	int pin_register_index;
	int ret;

	if (ZL3073X_IS_INPUT_PIN(zl3073x_pin->index)) {
		pin_register_index = ZL3073X_REG_MAP_INPUT_PIN_GET(zl3073x_pin->index);
		ret = zl3073x_dpll_set_priority_ref(zl3073x_dpll->zl3073x, zl3073x_dpll->index,
						pin_register_index, prio);
	} else {
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static int zl3073x_dpll_pin_phase_offset_get(const struct dpll_pin *pin, void *pin_priv,
				const struct dpll_device *dpll, void *dpll_priv,
				s64 *phase_offset,
				struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zl3073x_dpll = dpll_priv;
	struct zl3073x_pin *zl3073x_pin = pin_priv;
	u8 pin_register_index;
	int ret = 0;

	if (ZL3073X_IS_INPUT_PIN(zl3073x_pin->index)) {
		pin_register_index = ZL3073X_REG_MAP_INPUT_PIN_GET(zl3073x_pin->index);
		ret = zl3073x_dpll_phase_offset_get(zl3073x_dpll->zl3073x, zl3073x_dpll,
					pin_register_index, phase_offset);
	} else {
		/* Phase offset relative to output pins is not supported*/
		*phase_offset = 0;
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static int zl3073x_dpll_pin_phase_adjust_get(const struct dpll_pin *pin, void *pin_priv,
				const struct dpll_device *dpll, void *dpll_priv,
				s32 *phase_adjust,
				struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zl3073x_dpll = dpll_priv;
	struct zl3073x_pin *zl3073x_pin = pin_priv;
	int pin_register_index;
	int ret;

	if (ZL3073X_IS_INPUT_PIN(zl3073x_pin->index)) {
		pin_register_index = ZL3073X_REG_MAP_INPUT_PIN_GET(zl3073x_pin->index);
		ret = zl3073x_dpll_get_input_phase_adjust(zl3073x_dpll->zl3073x, pin_register_index, phase_adjust);
	} else {
		ret = zl3073x_dpll_get_output_phase_adjust(zl3073x_dpll->zl3073x, zl3073x_pin->index, phase_adjust);
	}

	return ret;
}

static int zl3073x_dpll_pin_phase_adjust_set(const struct dpll_pin *pin, void *pin_priv,
				const struct dpll_device *dpll, void *dpll_priv,
				const s32 phase_adjust,
				struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zl3073x_dpll = dpll_priv;
	struct zl3073x_pin *zl3073x_pin = pin_priv;
	int pin_register_index;
	int ret;

	if (ZL3073X_IS_INPUT_PIN(zl3073x_pin->index)) {
		pin_register_index = ZL3073X_REG_MAP_INPUT_PIN_GET(zl3073x_pin->index);
		ret = zl3073x_dpll_set_input_phase_adjust(zl3073x_dpll->zl3073x, pin_register_index, phase_adjust);
	} else {
		ret = zl3073x_dpll_set_output_phase_adjust(zl3073x_dpll->zl3073x, zl3073x_pin->index, phase_adjust);
	}

	return ret;
}

static int zl3073x_dpll_ffo_get(struct zl3073x *zl3073x, u8 dpll_index, u8 ref_index, s64 *ffo)
{
	u8 dpll_select_mask = (dpll_index) << DPLL_MEAS_REF_FREQ_MASK_SHIFT;
	u8 freq_meas_request = 0b11;
	u8 dpll_meas_ref_freq_ctrl;
	u8 freq_meas_enable = 0b1;
	s64 freq_offset_reg;
	u8 ref_select_mask;
	u8 freq_err[4];
	int ret;
	int val;

	mutex_lock(zl3073x->lock);

	ret = readx_poll_timeout_atomic(zl3073x_dpll_ref_freq_meas_op, zl3073x,
					val,
					!(REF_FREQ_MEAS_CTRL_MASK & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto err;

	/* Set the dpll mask and enable freq measurement */
	dpll_meas_ref_freq_ctrl = 0;
	dpll_meas_ref_freq_ctrl |= dpll_select_mask;
	dpll_meas_ref_freq_ctrl |= freq_meas_enable;
	zl3073x_write(zl3073x, DPLL_MEAS_REF_FREQ_CTRL, &dpll_meas_ref_freq_ctrl,
					sizeof(dpll_meas_ref_freq_ctrl));

	/* Set the reference mask */
	if (ref_index < 8) {
		ref_select_mask = BIT(ref_index);
		zl3073x_write(zl3073x, REF_FREQ_MEAS_MASK_3_0, &ref_select_mask, sizeof(ref_select_mask));
	} else {
		ref_select_mask = BIT(ref_index - 8);
		zl3073x_write(zl3073x, REF_FREQ_MEAS_MASK_4, &ref_select_mask, sizeof(ref_select_mask));
	}

	/* Request a read of the freq offset between the dpll and the reference */
	zl3073x_write(zl3073x, REF_FREQ_MEAS_CTRL, &freq_meas_request, sizeof(freq_meas_request));

	ret = readx_poll_timeout_atomic(zl3073x_dpll_ref_freq_meas_op, zl3073x,
					val,
					!(REF_FREQ_MEAS_CTRL_MASK & val),
					READ_SLEEP_US, READ_TIMEOUT_US);
	if (ret)
		goto err;

	zl3073x_read(zl3073x, DPLL_REF_FREQ_ERR(ref_index), freq_err, sizeof(freq_err));

	mutex_unlock(zl3073x->lock);

	/* register units for FFO are 2^-32 signed */
	freq_offset_reg = 0;
	freq_offset_reg |= ((s64)freq_err[3] << 0);
	freq_offset_reg |= ((s64)freq_err[2] << 8);
	freq_offset_reg |= ((s64)freq_err[1] << 16);
	freq_offset_reg |= ((s64)freq_err[0] << 24);


	/* Perform sign extension for a 32-bit signed value */
	if (freq_err[0] & 0x80)
		freq_offset_reg |= 0xFFFFFFFF00000000LL;

	*ffo = (s64)freq_offset_reg;

	return ret;

err:
	mutex_unlock(zl3073x->lock);
	*ffo = 0;

	return ret;
}

static int zl3073x_dpll_pin_ffo_get(const struct dpll_pin *pin, void *pin_priv,
		       const struct dpll_device *dpll, void *dpll_priv,
		       s64 *ffo, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zl3073x_dpll = dpll_priv;
	struct zl3073x_pin *zl3073x_pin = pin_priv;
	int pin_register_index;
	int ret;

	if (ZL3073X_IS_INPUT_PIN(zl3073x_pin->index)) {
		pin_register_index = ZL3073X_REG_MAP_INPUT_PIN_GET(zl3073x_pin->index);
		ret = zl3073x_dpll_ffo_get(zl3073x_dpll->zl3073x, zl3073x_dpll->index, pin_register_index, ffo);
	} else {
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static int zl3073x_dpll_pin_esync_set(const struct dpll_pin *pin, void *pin_priv,
			 const struct dpll_device *dpll, void *dpll_priv,
			 u64 freq, struct netlink_ext_ack *extack)
{
	return -EOPNOTSUPP;
}

static int zl3073x_dpll_pin_esync_get(const struct dpll_pin *pin, void *pin_priv,
			 const struct dpll_device *dpll, void *dpll_priv,
			 struct dpll_pin_esync *esync,
			 struct netlink_ext_ack *extack)
{
	return -EOPNOTSUPP;
}

static int zl3073x_dpll_lock_status_get(const struct dpll_device *dpll, void *dpll_priv,
			       enum dpll_lock_status *status,
			       enum dpll_lock_status_error *status_error,
			       struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zl3073x_dpll = dpll_priv;
	u8 raw_lock_status;

	raw_lock_status = zl3073x_dpll_raw_lock_status_get(zl3073x_dpll->zl3073x, zl3073x_dpll->index);
	*status = zl3073x_dpll_map_raw_to_manager_lock_status(zl3073x_dpll->zl3073x,
					zl3073x_dpll->index, raw_lock_status);

	return 0;
}

static int zl3073x_dpll_mode_get(const struct dpll_device *dpll, void *dpll_priv,
			enum dpll_mode *mode, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zl3073x_dpll = dpll_priv;
	u8 raw_mode;

	raw_mode = zl3073x_dpll_raw_mode_get(zl3073x_dpll->zl3073x, zl3073x_dpll->index);
	*mode = zl3073x_dpll_map_raw_to_manager_mode(raw_mode);

	return 0;
}

static const struct dpll_pin_ops zl3073x_dpll_pin_ops = {
	.direction_get      = zl3073x_dpll_pin_direction_get,
	.state_on_dpll_get  = zl3073x_dpll_pin_state_on_dpll_get,
	.frequency_get      = zl3073x_dpll_pin_frequency_get,
	.frequency_set      = zl3073x_dpll_pin_frequency_set,
	.direction_get      = zl3073x_dpll_pin_direction_get,
	.prio_get           = zl3073x_dpll_pin_prio_get,
	.prio_set           = zl3073x_dpll_pin_prio_set,
	.phase_offset_get   = zl3073x_dpll_pin_phase_offset_get,
	.phase_adjust_get   = zl3073x_dpll_pin_phase_adjust_get,
	.phase_adjust_set   = zl3073x_dpll_pin_phase_adjust_set,
	.ffo_get            = zl3073x_dpll_pin_ffo_get,
	.esync_set          = zl3073x_dpll_pin_esync_set,
	.esync_get          = zl3073x_dpll_pin_esync_get,
};

static const struct dpll_device_ops zl3073x_dpll_device_ops = {
	.lock_status_get    = zl3073x_dpll_lock_status_get,
	.mode_get           = zl3073x_dpll_mode_get,
};

static u16 zl3073x_dpll_chip_id_get(struct zl3073x *zl3073x)
{
	u16 chip_id;
	u8 buf[2];

	zl3073x_read(zl3073x, DPLL_CHIP_ID_REG, buf, sizeof(buf));

	chip_id = buf[0] + (buf[1] << 8);
	return chip_id;
}

static u64 zl3073x_dpll_clock_id_get(struct zl3073x *zl3073x)
{
	u64 clock_id = 0;
	u16 chip_id;

	chip_id = zl3073x_dpll_chip_id_get(zl3073x);

	/* Leave some bits if clock id for potential systems with multiple chips */
	clock_id = (chip_id << 16);

	return clock_id;
}

static struct dpll_pin_properties zl3073x_dpll_input_pin_properties_get(int pin_index)
{
	struct dpll_pin_properties input_pin_prop;

	input_pin_prop.board_label = input_pin_names[pin_index];
	input_pin_prop.type = input_dpll_pin_types[pin_index];
	input_pin_prop.capabilities = DPLL_PIN_CAPABILITIES_STATE_CAN_CHANGE |
					DPLL_PIN_CAPABILITIES_PRIORITY_CAN_CHANGE;
	input_pin_prop.freq_supported_num = ARRAY_SIZE(input_freq_ranges);
	input_pin_prop.freq_supported = input_freq_ranges;
	input_pin_prop.phase_range = phase_range;

	return input_pin_prop;
}

static struct dpll_pin_properties zl3073x_dpll_output_pin_properties_get(int pin_index)
{
	enum zl3073x_output_freq_type_t freq_type = output_freq_type_per_output[pin_index / 2];
	struct dpll_pin_properties output_pin_prop;

	output_pin_prop.board_label = output_pin_names[pin_index];
	output_pin_prop.type = output_dpll_pin_types[pin_index];
	output_pin_prop.capabilities = 0;

	if (freq_type == ZL3073X_SYNCE) {
		output_pin_prop.freq_supported = output_freq_range_synce;
		output_pin_prop.freq_supported_num = ARRAY_SIZE(output_freq_range_synce);
	} else if (freq_type == ZL3073X_PTP) {
		output_pin_prop.freq_supported = output_freq_range_ptp;
		output_pin_prop.freq_supported_num = ARRAY_SIZE(output_freq_range_ptp);
	} else if (freq_type == ZL3073X_25MHz) {
		output_pin_prop.freq_supported = output_freq_range_25MHz;
		output_pin_prop.freq_supported_num = ARRAY_SIZE(output_freq_range_25MHz);
	}
	output_pin_prop.phase_range = phase_range;

	return output_pin_prop;
}

static int zl3073x_dpll_register(struct zl3073x_dpll *zl3073x_dpll,
				enum dpll_type dpll_type, int dpll_index)
{
	struct dpll_device *dpll_device;
	u64 clock_id;
	int ret = 0;

	clock_id = zl3073x_dpll_clock_id_get(zl3073x_dpll->zl3073x);
	dpll_device = dpll_device_get(clock_id, dpll_index, THIS_MODULE);

	if (IS_ERR((dpll_device))) {
		ret = PTR_ERR(dpll_device);
	} else {
		zl3073x_dpll->index = dpll_index;
		zl3073x_dpll->dpll_device = dpll_device;

		ret = dpll_device_register(dpll_device, dpll_type, &zl3073x_dpll_device_ops, zl3073x_dpll);
	}

	return ret;
}

static void zl3073x_dpll_unregister(struct zl3073x_dpll *zl3073x_dpll)
{
	dpll_device_unregister(zl3073x_dpll->dpll_device, &zl3073x_dpll_device_ops, zl3073x_dpll);
	dpll_device_put(zl3073x_dpll->dpll_device);
}

static int zl3073x_pin_register(struct zl3073x_dpll *zl3073x_dpll,
				struct zl3073x_pin *zl3073x_pin, int pin_index)
{
	u64 clock_id = zl3073x_dpll_clock_id_get(zl3073x_dpll->zl3073x);
	struct dpll_pin_properties pin_properties;
	enum zl3073x_pin_type pin_type;
	int pin_register_index;
	int ret = 0;

	if (ZL3073X_IS_INPUT_PIN(pin_index)) {
		pin_register_index = ZL3073X_REG_MAP_INPUT_PIN_GET(pin_index);
		pin_properties = zl3073x_dpll_input_pin_properties_get(pin_register_index);
		pin_type = ZL3073X_SINGLE_ENDED;
	} else {
		pin_properties = zl3073x_dpll_output_pin_properties_get(pin_index);
		pin_type = zl3073x_output_pin_type[pin_index / 2];
	}

	struct dpll_pin *dpll_pin = dpll_pin_get(clock_id, pin_index, THIS_MODULE, &pin_properties);

	if (IS_ERR((dpll_pin))) {
		ret = PTR_ERR(dpll_pin);
	} else {
		zl3073x_pin->index = pin_index;
		zl3073x_pin->dpll_pin = dpll_pin;
		zl3073x_pin->pin_type = pin_type;

		ret = dpll_pin_register(zl3073x_dpll->dpll_device, dpll_pin,
						&zl3073x_dpll_pin_ops, zl3073x_pin);
	}

	return ret;
}

static void zl3073x_pin_unregister(struct zl3073x *zl3073x, struct zl3073x_pin *zl3073x_pin)
{
	struct zl3073x_dpll *zl3073x_dpll;

	/* unregister each pin on each dpll */
	for (int i = 0; i < ZL3073X_MAX_DPLLS; i++) {
		zl3073x_dpll = &zl3073x->dpll[i];
		dpll_pin_unregister(zl3073x_dpll->dpll_device, zl3073x_pin->dpll_pin,
					&zl3073x_dpll_pin_ops, zl3073x_pin);
	}
	dpll_pin_put(zl3073x_pin->dpll_pin);
}

static int zl3073x_register_all_dplls(struct zl3073x *zl3073x)
{
	struct zl3073x_dpll *zl3073x_dpll;
	enum dpll_type dpll_type;
	int ret = 0;
	int i;

	for (i = 0; i < ZL3073X_MAX_DPLLS; i++) {
		dpll_type = zl3073x_dpll_type[i];
		zl3073x_dpll = &zl3073x->dpll[i];
		zl3073x_dpll->zl3073x = zl3073x;
		ret = zl3073x_dpll_register(zl3073x_dpll, dpll_type, i);

		if (ret)
			goto err;
	}

	return ret;

err:
	while (i >= 0) {
		zl3073x_dpll = &zl3073x->dpll[i];
		zl3073x_dpll_unregister(zl3073x_dpll);
		i--;
	}
	return ret;
}

static void zl3073x_unregister_all_dplls(struct zl3073x *zl3073x)
{
	struct zl3073x_dpll *zl3073x_dpll;

	for (int i = 0; i < ZL3073X_MAX_DPLLS; i++) {
		zl3073x_dpll = &zl3073x->dpll[i];
		zl3073x_dpll_unregister(zl3073x_dpll);
	}
}

static int zl3073x_register_all_pins(struct zl3073x *zl3073x)
{
	struct zl3073x_dpll *zl3073x_dpll;
	struct zl3073x_pin *zl3073x_pin;
	int pin_index;
	int ret = 0;
	int i;

	for (i = 0; i < ZL3073X_MAX_PINS; i++) {
		zl3073x_pin = &zl3073x->pin[i];
		pin_index = i;

		/* Register each pin on each dpll */
		for (int j = 0; j < ZL3073X_MAX_DPLLS; j++) {
			zl3073x_dpll = &zl3073x->dpll[j];
			ret = zl3073x_pin_register(zl3073x_dpll, zl3073x_pin, pin_index);

			if (ret)
				goto err;
		}
	}

	return ret;

err:
	while (i >= 0) {
		zl3073x_pin = &zl3073x->pin[i];
		zl3073x_pin_unregister(zl3073x, zl3073x_pin);
		i--;
	}
	return ret;
}

static void zl3073x_unregister_all_pins(struct zl3073x *zl3073x)
{
	struct zl3073x_pin *zl3073x_pin;

	for (int i = 0; i < ZL3073X_MAX_PINS; i++) {
		zl3073x_pin = &zl3073x->pin[i];
		zl3073x_pin_unregister(zl3073x, zl3073x_pin);
	}
}

static int zl3073x_dpll_init(struct zl3073x *zl3073x)
{
	int ret = 0;

	ret = zl3073x_register_all_dplls(zl3073x);

	if (ret != 0)
		goto out;

	ret = zl3073x_register_all_pins(zl3073x);

	if (ret != 0)
		goto out;

out:
	return ret;
}

static int zl3073x_dpll_init_fine_phase_adjust(struct zl3073x *zl3073x)
{
	u8 phase_shift_data[] = { 0xFF, 0xFF };
	u8 phase_shift_intvl = 0x01;
	u8 phase_shift_mask = 0x1F;
	u8 phase_shift_ctrl = 0x01;
	int ret;

	ret = zl3073x_write(zl3073x, DPLL_SYNTH_PHASE_SHIFT_MASK,
			    &phase_shift_mask, sizeof(phase_shift_mask));

	if (ret)
		return ret;

	ret = zl3073x_write(zl3073x, DPLL_SYNTH_PHASE_SHIFT_INTVL,
			    &phase_shift_intvl, sizeof(phase_shift_intvl));

	if (ret)
		return ret;

	ret = zl3073x_write(zl3073x, DPLL_SYNTH_PHASE_SHIFT_DATA,
			    phase_shift_data, sizeof(phase_shift_data));

	if (ret)
		return ret;

	ret = zl3073x_write(zl3073x, DPLL_SYNTH_PHASE_SHIFT_CTRL,
			    &phase_shift_ctrl, sizeof(phase_shift_ctrl));

	return ret;
}

static int zl3073x_ptp_init(struct zl3073x *zl3073x, u8 index)
{
	struct zl3073x_dpll *dpll = &zl3073x->dpll[index];

	for (int i = 0; i < ZL3073X_MAX_OUTPUT_PINS; i++) {
		struct ptp_pin_desc *p = &dpll->pins[i];

		snprintf(p->name, sizeof(p->name), "pin%d", i);
		p->index = i;
		p->func = PTP_PF_NONE;
		p->chan = 0;
	}

	dpll->index = index;
	dpll->zl3073x = zl3073x;
	dpll->info = zl3073x_ptp_clock_info;
	dpll->info.pin_config = dpll->pins;
	dpll->clock = ptp_clock_register(&dpll->info, zl3073x->dev);
	if (IS_ERR(dpll->clock))
		return PTR_ERR(dpll->clock);

	return 0;
}

static const char *_zl3073x_firmware_get_line(const char *data,
					      size_t line_number)
{
	for (int i = 0; i < line_number; ++i) {
		data = strchr(data, '\n');
		if (!data)
			return NULL;
		data += 1;
	}

	return data;
}

static int _zl3073x_firmware_parse_line(struct zl3073x *zl3073x,
					const char *line)
{
	const char *tmp = line;
	int err = 0;
	u8 val = 0;
	char *endp;
	u32 delay;
	u16 addr;

	mutex_lock(zl3073x->lock);
	switch (tmp[0]) {
	case 'X':
		/* The line looks like this:
		 * X , ADDR , VAL
		 * Where:
		 *  - X means that is a command that needs to be executed
		 *  - ADDR represents the addr and is always 2 bytes and the
		 *         value is in hex, for example 0x0232
		 *  - VAL represents the value that is written and is always 1
		 *        byte and the value is in hex, for example 0x12
		 */
		tmp += ZL3073X_FW_COMMAND_SIZE;

		tmp += ZL3073X_FW_WHITESPACES_SIZE;
		addr = simple_strtoul(tmp, &endp, 16);

		tmp = endp;
		tmp += ZL3073X_FW_WHITESPACES_SIZE;
		val = simple_strtoul(tmp, &endp, 16);

		err = zl3073x_write(zl3073x, addr, &val, 1);
		break;
	case 'W':
		/* The line looks like this:
		 * W , DELAY
		 * Where:
		 *  - W means that is a wait command
		 *  - DELAY represents the delay in microseconds and the value
		 *    is in decimal
		 */
		tmp += ZL3073X_FW_COMMAND_SIZE;

		tmp += ZL3073X_FW_WHITESPACES_SIZE;
		delay = simple_strtoul(tmp, &endp, 10);

		usleep_range(delay / 2, delay);
		break;
	default:
		break;
	}
	mutex_unlock(zl3073x->lock);

	return err;
}

static int zl3073x_firmware_load(struct zl3073x *zl3073x)
{
	char fname[128] = ZL3073X_FW_FILENAME;
	const struct firmware *fw;
	size_t line_number = 0;
	const char *line;
	int err = 0;

	err = request_firmware(&fw, fname, zl3073x->dev);
	if (err)
		return err;

	while (true) {
		line = _zl3073x_firmware_get_line(fw->data, line_number);
		if (!line)
			goto out;

		line_number += 1;

		/* Skip comment lines */
		if (line[0] == ';')
			continue;

		err = _zl3073x_firmware_parse_line(zl3073x, line);
		if (err)
			goto out;
	}

out:
	release_firmware(fw);
	return err;
}

static int zl3073x_probe(struct platform_device *pdev)
{
	struct microchip_dpll_ddata *ddata = dev_get_drvdata(pdev->dev.parent);
	struct zl3073x *zl3073x;
	int err;

	zl3073x = devm_kzalloc(&pdev->dev, sizeof(struct zl3073x), GFP_KERNEL);
	if (!zl3073x)
		return -ENOMEM;

	zl3073x->dev = &pdev->dev;
	zl3073x->mfd = pdev->dev.parent;
	zl3073x->lock = &ddata->lock;
	zl3073x->regmap = ddata->regmap;

	zl3073x_firmware_load(zl3073x);

#if IS_ENABLED(CONFIG_PTP_1588_CLOCK_ZL3073X)
	err = zl3073x_ptp_init(zl3073x, ZL3073X_PTP_CLOCK_DPLL);
	if (err)
		return err;
#endif

#if IS_ENABLED(CONFIG_DPLL)
	err = zl3073x_dpll_init(zl3073x);
	if (err)
		return err;
#endif

	platform_set_drvdata(pdev, zl3073x);

	/* Initial firmware fine phase correction */
	zl3073x_dpll_init_fine_phase_adjust(zl3073x);

	return 0;
}

static void zl3073x_remove(struct platform_device *pdev)
{
	struct zl3073x *zl3073x = platform_get_drvdata(pdev);

#if IS_ENABLED(CONFIG_PTP_1588_CLOCK_ZL3073X)
	ptp_clock_unregister(zl3073x->dpll[ZL3073X_PTP_CLOCK_DPLL].clock);
#endif

#if IS_ENABLED(CONFIG_DPLL)
	/* Unregister all pins and dpll */
	zl3073x_unregister_all_pins(zl3073x);
	zl3073x_unregister_all_dplls(zl3073x);
#endif
}

static struct platform_driver zl3073x_driver = {
	.driver = {
		.name = "microchip,zl3073x",
		.of_match_table = zl3073x_match,
	},
	.probe = zl3073x_probe,
	.remove	= zl3073x_remove,
};

module_platform_driver(zl3073x_driver);

MODULE_DESCRIPTION("Driver for zl3073x clock devices");
MODULE_LICENSE("GPL");
