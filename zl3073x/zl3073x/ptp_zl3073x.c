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

#include "ptp_private.h"
#include <linux/dpll.h>

#define DPLL_MON_STATUS(index)			(0x110 + (index))
#define DPLL_MON_STATUS_HO_READY_GET(val)	((val & GENMASK(2,2)) >> 2)
#define DPLL_LOCK_STATUS(index)			(0x130 + (index))
#define DPLL_LOCK_STATUS_GET(val)		(val & GENMASK(6,4) >> 4)
#define DPLL_MODE_REFSEL(index)			(0x284 + (index) * 0x4)
#define DPLL_MODE_REFSEL_MODE_GET(val)		(val & GENMASK(2, 0))

#define DPLL_TIE_CTRL				0x2b0
#define DPLL_TIE_CTRL_MASK			GENMASK(2, 0)
#define DPLL_TIE_CTRL_MASK_REG			0x2b1
#define DPLL_TIE_CTRL_OPERATION			4
#define DPLL_TIE_CTRL_SIZE			1

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



#define DPLL_DPLL_MB_MASK			0x602
#define DPLL_DPLL_MB_MASK_SIZE			2
#define DPLL_DPLL_MB_SEM			0x604
#define DPLL_DPLL_MB_SEM_SIZE			1
#define DPLL_DPLL_MB_SEM_RD			BIT(1)


#define DPLL_REF_PRIORITY(refId)				(0x652 + (refId/2))
#define DPLL_REF_PRIORITY_GET_UPPER(data)		(((data) & GENMASK(7,4)) >> 4)
#define DPLL_REF_PRIORITY_GET_LOWER(data)		((data) & GENMASK(3,0))
#define DPLL_REF_PRIORITY_GET(data, refId)		(((refId) % 2 == 0) ? ZLS3073X_DPLL_REF_PRIORITY_GET_LOWER(data) : \
                                                                              ZLS3073X_DPLL_REF_PRIORITY_GET_UPPER(data))

/*#define DPLL_GET_PRIORITY(index)		(0x652 + (index))
#define DPLL_GET_PRIORITY_REFP(val)		((val & GENMASK(3,0)))
#define DPLL_GET_PRIORITY_REFN(val)		((val & GENMASK(7,4)) >> 4)
#define DPLL_SET_PRIORITY_REFP(current_priority, refp)	(((current_priority) & ~GENMASK(3, 0)) | ((refp) & GENMASK(3, 0)))
#define DPLL_SET_PRIORITY_REFN(current_priority, refn)	(((current_priority) & ~GENMASK(7, 4)) | (((refn) & GENMASK(3, 0)) << 4))*/

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
#define DPLL_OUTPUT_GPO_EN			0x724
#define DPLL_OUTPUT_GPO_EN_SIZE			1

#define ZL3073X_1PPM_FORMAT		281474976

#define ZL3073X_MAX_DPLLS		2
#define ZL3073X_MAX_OUTPUTS		20

#define READ_SLEEP_US			10
#define READ_TIMEOUT_US			100000000

#define ZL3073X_FW_FILENAME		"zl3073x.mfg"
#define ZL3073X_FW_WHITESPACES_SIZE	3
#define ZL3073X_FW_COMMAND_SIZE		1

#define ZL3073X_P_PIN(pin)		((pin) % 2 == 0)
#define ZL3073X_N_PIN(pin)		(!ZL3073X_P_PIN(pin))

static const struct of_device_id zl3073x_match[] = {
	{ .compatible = "microchip,zl80732-phc" },
	{ .compatible = "microchip,zl30732b-phc" },
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

enum zl3073x_dpll_state_t{
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

struct zl3073x_dpll {
	struct zl3073x		*zl3073x;
	u8			index;

	struct ptp_clock_info	info;
	struct ptp_clock	*clock;
	struct ptp_pin_desc	pins[ZL3073X_MAX_OUTPUTS];

	u16			perout_mask;
};

struct zl3073x {
	struct device		*dev;
	struct mutex		*lock;
	struct regmap		*regmap;
	struct device		*mfd;

	struct zl3073x_dpll	dpll[ZL3073X_MAX_DPLLS];
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
		return ret;;

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
	printk("DPLL RAW MODE: DPLL index %d, DPLL RAW mode %d\n", dpll_index, DPLL_MODE_REFSEL_MODE_GET(mode));
    
	return DPLL_MODE_REFSEL_MODE_GET(mode);
}

static int zl3073x_dpll_map_raw_to_manager_mode_status(int raw_mode) 
{
	printk("MAP_RAW_TO_MANAGER_MODE\n");
	
	switch (raw_mode) {
	case ZL3073X_MODE_HOLDOVER:
	case ZL3073X_MODE_REFLOCK:
		printk("DPLL mode HOLDOVER/REFLOCK, print MANUAL\n");
		return DPLL_MODE_MANUAL;
	case ZL3073X_MODE_AUTO_LOCK:
		printk("DPLL mode LOCK, print AUTOMATIC\n");
		return DPLL_MODE_AUTOMATIC;
	case ZL3073X_MODE_FREERUN:
	case ZL3073X_MODE_NCO:
	default:
		printk("DPLL mode FREERUN/NCO, print INVALID\n");
		return -EINVAL;
	}
}


static int zl3073x_dpll_lock_status_get(struct zl3073x *zl3073x, int dpll_index)
{
	u8 dpll_status;
	zl3073x_read(zl3073x, DPLL_LOCK_STATUS(dpll_index), &dpll_status, sizeof(dpll_status));
	printk("DPLL RAW Lock State: DPLL index %d, DPLL RAW STATE %d\n", dpll_index, DPLL_LOCK_STATUS_GET(dpll_status));
	
	return DPLL_LOCK_STATUS_GET(dpll_status);
}

static int zl3073x_dpll_map_raw_to_manager_lock_status(struct zl3073x *zl3073x, int dpll_index, u8 dpll_status)
{
	printk("MAP_RAW_TO_MANAGER_LOCK_STATE\n");
	u8 dpll_mon_status;
    u8 ho_ready;
	
    zl3073x_read(zl3073x, DPLL_MON_STATUS(dpll_index), &dpll_mon_status, sizeof(dpll_mon_status));
	ho_ready = DPLL_MON_STATUS_HO_READY_GET(dpll_mon_status);
	
	switch (dpll_status) {
	case ZLS3073X_DPLL_STATE_FREERUN:
	case ZLS3073X_DPLL_STATE_FAST_LOCK:
	case ZLS3073X_DPLL_STATE_ACQUIRING:
		printk("DPLL state is FREERUN/FASTLOCK/ACQUIRING, print UNLOCKED\n");
		return DPLL_LOCK_STATUS_UNLOCKED;
	case ZLS3073X_DPLL_STATE_HOLDOVER:
		printk("DPLL state is HOLDOVER print DPLL_LOCK_STATUS_HOLDOVER\n");
		return DPLL_LOCK_STATUS_HOLDOVER;
	case ZLS3073X_DPLL_STATE_LOCK:
		printk("DPLL state is LOCK, print DPLL_LOCK_STATUS_LOCKED\n");
		if (ho_ready) {
			return DPLL_LOCK_STATUS_LOCKED_HO_ACQ;
		} else {
			return DPLL_LOCK_STATUS_LOCKED;
		}
	default:
		return -EINVAL;
	}
}


static int zl3073x_dpll_get_priority_ref(struct zl3073x *zl3073x, u8 dpll_index, u8 refId)
{
	u8 refpriority;
	u8 get_priority;

	u8 buf[3];
	int ret;
	int val;

	/* Select the synth */
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
		return ret;

	printk("priority_get() \n");
	zl3073x_read(zl3073x, DPLL_REF_PRIORITY(refId), &get_priority, sizeof(get_priority));
	refpriority = DPLL_REF_PRIORITY_GET(get_priority, refId);
	printk("priority_get_ref() %d\n", refpriority);

	mutex_unlock(zl3073x->lock);
	
	return refpriority;
	
}

static int zl3073x_dpll_set_priority_refp(struct zl3073x *zl3073x, int dpll_index, u8 refp)
{
	u8 current_priority;
	u8 new_priority;

	if (refp > 0xF) {
		printk("Invalid refp value: %d\n", refp);
		return -EINVAL; // Invalid argument error
	}

	printk("priority_set_refp() refp = %d\n", refp);

	zl3073x_read(zl3073x, DPLL_GET_PRIORITY(dpll_index), &current_priority, sizeof(current_priority));

	new_priority = DPLL_SET_PRIORITY_REFP(current_priority, new_priority);
	
	zl3073x_write(zl3073x, DPLL_GET_PRIORITY(dpll_index), &new_priority, sizeof(new_priority));
	
	printk("priority_set_refp() new_priority = 0x%02X\n", new_priority);

	return 0;
}

static int zl3073x_dpll_get_priority_refn(struct zl3073x *zl3073x, int dpll_index)
{
	u8 refn;
	u8 get_priority;
	
	printk("priority_get() \n");
	zl3073x_read(zl3073x, DPLL_GET_PRIORITY(dpll_index), &get_priority, sizeof(get_priority));
	refn = DPLL_GET_PRIORITY_REFN(get_priority);
	printk("priority_get_refn() %d\n", refn);
	
	return refn;
}

static int zl3073x_dpll_set_priority_refn(struct zl3073x *zl3073x, int dpll_index, u8 refn)
{
	u8 current_priority;
	u8 new_priority;
	
	if (refn > 0xF) {
		printk("Invalid refn value: %d\n", refn);
		return -EINVAL; // Invalid argument error
	}
	
	printk("priority_set_refn() refn = %d\n", refn);
	
	zl3073x_read(zl3073x, DPLL_GET_PRIORITY(dpll_index), &current_priority, sizeof(current_priority));

	new_priority = DPLL_SET_PRIORITY_REFP(current_priority, new_priority)
	
	zl3073x_write(zl3073x, DPLL_GET_PRIORITY(dpll_index), &new_priority, sizeof(new_priority));

	printk("priority_set_refn() new_priority = 0x%02X\n", new_priority);
	
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
	if (pin == -1 || pin >= ZL3073X_MAX_OUTPUTS)
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
	if (pin == -1 || pin >= ZL3073X_MAX_OUTPUTS)
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
		return -1;
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
		return -1;
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
	.n_per_out	= ZL3073X_MAX_OUTPUTS,
	.n_ext_ts	= ZL3073X_MAX_OUTPUTS,
	.n_pins		= ZL3073X_MAX_OUTPUTS,
};

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

	for (int i = 0; i < ZL3073X_MAX_OUTPUTS; i++) {
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

static bool zl3073x_dpll_nco_mode(struct zl3073x *zl3073x, int dpll_index)
{
	u8 mode;

	zl3073x_read(zl3073x, DPLL_MODE_REFSEL(dpll_index), &mode, sizeof(mode));
	return DPLL_MODE_REFSEL_MODE_GET(mode) == ZL3073X_MODE_NCO;
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

	for (size_t i = 0; i < ZL3073X_MAX_DPLLS; ++i) {
		if (!zl3073x_dpll_nco_mode(zl3073x, i))
			continue;

		err = zl3073x_ptp_init(zl3073x, i);
		if (err)
			return err;
	}

	platform_set_drvdata(pdev, zl3073x);

	/* Initial firmware fine phase correction */
	zl3073x_dpll_init_fine_phase_adjust(zl3073x);

	return 0;
}

static int zl3073x_remove(struct platform_device *pdev)
{
	struct zl3073x *zl3073x = platform_get_drvdata(pdev);

	for (int i = 0; i < ZL3073X_MAX_DPLLS; ++i) {
		if (!zl3073x_dpll_nco_mode(zl3073x, i))
			continue;

		ptp_clock_unregister(zl3073x->dpll[i].clock);
	}

	return 0;
}

static struct platform_driver zl3073x_driver = {
	.driver = {
		.name = "microchip,zl3073x-phc",
		.of_match_table = zl3073x_match,
	},
	.probe = zl3073x_probe,
	.remove	= zl3073x_remove,
};

module_platform_driver(zl3073x_driver);

MODULE_DESCRIPTION("Driver for zl3073x clock devices");
MODULE_LICENSE("GPL");
