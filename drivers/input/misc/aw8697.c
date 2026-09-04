// SPDX-License-Identifier: GPL-2.0-only
/*
 * AW8697 / AW86927 LRA haptic driver
 *
 * Ported from the Android vendor driver
 * (crdroid-16-xaga/kernel/xiaomi/mt6895/drivers/input/misc/aw8697_haptic/),
 * Copyright (c) 2021 AWINIC Technology CO., LTD.
 *
 * Trimmed for mainline: RAM-mode playback of the firmware-predefined
 * waveforms (effects 0..effect_id_boundary-1) via FF_PERIODIC/FF_CUSTOM
 * plus FF_CONSTANT/FF_RUMBLE (RAM loop) and FF_GAIN. RTP streaming, sysfs, the misc
 * device, F0/offset calibration and the trigger/haptic-audio paths are not
 * ported.
 *
 * The xaga board is dual-BOM: it may carry either an AW8697 or an AW86927.
 * Both are detected by reading reg 0x00 (0x97 = AW8697) or regs 0x57/0x58
 * (0x9270 = AW86927); the register maps differ so each gets its own
 * init/play path. Both use the same RAM firmware (aw8697_haptic.bin)
 * loaded into SRAM via request_firmware() at probe time.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define AW8697_CHIP_ID			0x97
#define AW86927_CHIP_ID			0x9270
#define AW86927_REG_IDH			0x57
#define AW86927_REG_IDL			0x58

#define AW8697_REG_ID			0x00
#define AW8697_REG_SYSCTRL		0x04
#define AW8697_REG_GO			0x05
#define AW8697_REG_WAVSEQ1		0x07
#define AW8697_REG_WAVLOOP1		0x0f
#define AW8697_REG_PWMPRC		0x2d
#define AW8697_REG_PWMDBG		0x2e
#define AW8697_REG_BSTDBG1		0x31
#define AW8697_REG_BSTDBG2		0x32
#define AW8697_REG_BSTDBG3		0x33
#define AW8697_REG_BSTCFG		0x34
#define AW8697_REG_ANADBG		0x35
#define AW8697_REG_ANACTRL		0x36
#define AW8697_REG_DATDBG		0x39
#define AW8697_REG_BSTDBG4		0x3a
#define AW8697_REG_RAMADDRH		0x40
#define AW8697_REG_RAMADDRL		0x41
#define AW8697_REG_RAMDATA		0x42
#define AW8697_REG_GLB_STATE		0x46
#define AW8697_REG_BST_AUTO		0x47
#define AW8697_REG_F_PRE_H		0x49
#define AW8697_REG_F_PRE_L		0x4a
#define AW8697_REG_TSET			0x4d
#define AW8697_REG_TRIM_LRA		0x5b
#define AW8697_REG_R_SPARE		0x5d
#define AW8697_REG_DETCTRL		0x5f
#define AW8697_REG_ADCTEST		0x66

/* AW8697 SYSCTRL */
#define AW8697_SYSCTRL_WAVDAT_MODE_MASK		GENMASK(7, 6)
#define AW8697_SYSCTRL_RAMINIT_MASK		BIT(5)
#define AW8697_SYSCTRL_PLAY_MODE_MASK		GENMASK(3, 2)
#define AW8697_SYSCTRL_PLAY_MODE_CONT		BIT(3)
#define AW8697_SYSCTRL_PLAY_MODE_RTP		BIT(2)
#define AW8697_SYSCTRL_PLAY_MODE_RAM		0
#define AW8697_SYSCTRL_BST_MODE_MASK		BIT(1)
#define AW8697_SYSCTRL_BST_MODE_BOOST		BIT(1)
#define AW8697_SYSCTRL_WORK_MODE_MASK		BIT(0)
#define AW8697_SYSCTRL_STANDBY			BIT(0)
#define AW8697_SYSCTRL_ACTIVE			0

/* AW8697 GO */
#define AW8697_GO_MASK				BIT(0)
#define AW8697_GO_ENABLE			BIT(0)

/* AW8697 WAVLOOP */
#define AW8697_WAVLOOP_SEQN_MASK		GENMASK(7, 4)
#define AW8697_WAVLOOP_SEQNP1_MASK		GENMASK(3, 0)
#define AW8697_WAVLOOP_INFINITELY		0x0f

/* AW8697 PWMDBG */
#define AW8697_PWMDBG_PWM_MODE_MASK		GENMASK(7, 5)
#define AW8697_PWMDBG_PWM_24K			BIT(6)

/* AW8697 BSTCFG */
#define AW8697_BSTCFG_PEAKCUR_MASK		GENMASK(2, 0)
#define AW8697_BSTCFG_PEAKCUR_3P5A		5

/* AW8697 ANADBG */
#define AW8697_ANADBG_IOC_MASK			GENMASK(3, 2)
#define AW8697_ANADBG_IOC_4P65A			GENMASK(3, 2)

/* AW8697 ANACTRL */
#define AW8697_ANACTRL_LRA_SRC_MASK		BIT(5)
#define AW8697_ANACTRL_LRA_SRC_REG		BIT(5)

/* AW8697 BSTDBG4 */
#define AW8697_BSTDBG4_BSTVOL_MASK		GENMASK(5, 1)

/* AW8697 DETCTRL */
#define AW8697_DETCTRL_PROTECT_MASK		BIT(5)
#define AW8697_DETCTRL_PROTECT_NO_ACTION	BIT(5)

/* AW8697 PWMPRC */
#define AW8697_PWMPRC_PRC_MASK			BIT(7)

/* AW8697 BST_AUTO */
#define AW8697_BST_AUTO_AUTOSW_MASK		BIT(2)
#define AW8697_BST_AUTO_MANUAL_BOOST		0

/* AW8697 ADCTEST */
#define AW8697_ADCTEST_VBAT_MODE_MASK		BIT(6)
#define AW8697_ADCTEST_VBAT_HW_COMP		BIT(6)

/* AW86927 registers */
#define AW86927_REG_RSTCFG			0x00
#define AW86927_REG_PLAYCFG1			0x06
#define AW86927_REG_PLAYCFG2			0x07
#define AW86927_REG_PLAYCFG3			0x08
#define AW86927_REG_PLAYCFG4			0x09
#define AW86927_REG_WAVCFG1			0x0a
#define AW86927_REG_WAVCFG9			0x12
#define AW86927_REG_RTPCFG1			0x2d
#define AW86927_REG_RTPCFG2			0x2e
#define AW86927_REG_RTPCFG3			0x2f
#define AW86927_REG_RTPCFG4			0x30
#define AW86927_REG_RTPCFG5			0x31
#define AW86927_REG_GLBRD5			0x3f
#define AW86927_REG_RAMADDRH			0x40
#define AW86927_REG_RAMADDRL			0x41
#define AW86927_REG_RAMDATA			0x42
#define AW86927_REG_SYSCTRL3			0x45
#define AW86927_REG_SYSCTRL4			0x46
#define AW86927_REG_SYSCTRL5			0x47
#define AW86927_REG_PWMCFG1			0x48
#define AW86927_REG_PWMCFG2			0x49
#define AW86927_REG_PWMCFG3			0x4a
#define AW86927_REG_PWMCFG4			0x4b
#define AW86927_REG_VBATCTRL			0x4c
#define AW86927_REG_DETCFG2			0x4e
#define AW86927_REG_TMCFG			0x5b
#define AW86927_REG_CONTCFG1			0x18
#define AW86927_REG_CONTCFG5			0x1c
#define AW86927_REG_CONTCFG10			0x21
#define AW86927_REG_CONTCFG13			0x24
#define AW86927_REG_ANACFG12			0x71
#define AW86927_REG_ANACFG13			0x72
#define AW86927_REG_ANACFG15			0x74
#define AW86927_REG_ANACFG16			0x75

/* AW86927 PLAYCFG1 */
#define AW86927_PLAYCFG1_BST_MODE_MASK		BIT(7)
#define AW86927_PLAYCFG1_BST_MODE		BIT(7)
#define AW86927_PLAYCFG1_BST_VOUT_MASK		GENMASK(6, 0)
#define AW86927_BST_VOUT_10P5V			0x70
#define AW86927_BST_VOUT_6V			0x28

/* AW86927 PLAYCFG3 */
#define AW86927_PLAYCFG3_AUTO_BST_MASK		BIT(4)
#define AW86927_PLAYCFG3_PLAY_MODE_MASK		GENMASK(1, 0)
#define AW86927_PLAYCFG3_PLAY_MODE_STOP		GENMASK(1, 0)
#define AW86927_PLAYCFG3_PLAY_MODE_CONT		BIT(1)
#define AW86927_PLAYCFG3_PLAY_MODE_RTP		BIT(0)
#define AW86927_PLAYCFG3_PLAY_MODE_RAM		0

/* AW86927 PLAYCFG4 */
#define AW86927_PLAYCFG4_STOP_MASK		BIT(1)
#define AW86927_PLAYCFG4_STOP_ON		BIT(1)
#define AW86927_PLAYCFG4_GO_MASK		BIT(0)
#define AW86927_PLAYCFG4_GO_ON			BIT(0)

/* AW86927 WAVCFG9 loop */
#define AW86927_WAVLOOP_SEQ_ODD_MASK		GENMASK(7, 4)
#define AW86927_WAVLOOP_SEQ_EVEN_MASK		GENMASK(3, 0)
#define AW86927_WAVLOOP_INFINITELY		0x0f

/* AW86927 GLBRD5 */
#define AW86927_GLBRD5_STATE_MASK		GENMASK(3, 0)
#define AW86927_GLBRD5_STATE_STANDBY		0

/* AW86927 RTPCFG1 */
#define AW86927_RTPCFG1_BASE_ADDR_H_MASK	GENMASK(4, 0)
#define AW86927_RAMADDRH_MASK			GENMASK(4, 0)

/* AW86927 SYSCTRL3 */
#define AW86927_SYSCTRL3_STANDBY_MASK		BIT(5)
#define AW86927_SYSCTRL3_STANDBY_ON		BIT(5)
#define AW86927_SYSCTRL3_EN_RAMINIT_MASK	BIT(2)
#define AW86927_SYSCTRL3_EN_RAMINIT_ON		BIT(2)

/* AW86927 SYSCTRL4 */
#define AW86927_SYSCTRL4_WAVDAT_MODE_MASK	GENMASK(6, 5)
#define AW86927_SYSCTRL4_WAVDAT_24K		0
#define AW86927_SYSCTRL4_GAIN_BYPASS_MASK	BIT(0)
#define AW86927_SYSCTRL4_GAIN_CHANGEABLE	BIT(0)

/* AW86927 SYSCTRL5 */
#define AW86927_SYSCTRL5_INIT_VAL		0x5a

/* AW86927 PWMCFG1/2/3 */
#define AW86927_PWMCFG1_PRC_EN_MASK		BIT(7)
#define AW86927_PWMCFG1_PRCTIME_MASK		GENMASK(6, 0)
#define AW86927_PWMCFG2_PRCT_MODE_MASK		BIT(6)
#define AW86927_PWMCFG2_PRCT_MODE_VALID		0
#define AW86927_PWMCFG3_PR_EN_MASK		BIT(7)
#define AW86927_PWMCFG3_PRLVL_MASK		GENMASK(6, 0)
#define AW86927_PWMCFG1_INIT_VAL		0xa0

/* AW86927 VBATCTRL */
#define AW86927_VBATCTRL_VBAT_MODE_MASK		BIT(6)
#define AW86927_VBATCTRL_VBAT_MODE_SW		0

/* AW86927 DETCFG2 */
#define AW86927_DETCFG2_D2S_GAIN_MASK		GENMASK(2, 0)

/* AW86927 TMCFG */
#define AW86927_TMCFG_UNLOCK			0x7d
#define AW86927_TMCFG_LOCK			0x00

/* AW86927 CONTCFG1/5/10/13 */
#define AW86927_CONTCFG1_BRK_BST_MD_MASK	BIT(6)
#define AW86927_CONTCFG5_BST_BRK_GAIN_MASK	GENMASK(7, 4)
#define AW86927_CONTCFG5_BRK_GAIN_MASK		GENMASK(3, 0)
#define AW86927_CONTCFG10_BRK_TIME_MASK		GENMASK(7, 0)
#define AW86927_CONTCFG13_TSET_MASK		GENMASK(7, 4)
#define AW86927_CONTCFG13_BEME_SET_MASK		GENMASK(3, 0)

/* AW86927 ANACFG */
#define AW86927_ANACFG12_BST_SKIP_MASK		BIT(7)
#define AW86927_ANACFG12_BST_SKIP_SHUTDOWN	BIT(7)
#define AW86927_ANACFG13_PEAKCUR_MASK		GENMASK(7, 4)
#define AW86927_ANACFG13_PEAKCUR_3P45A		(6 << 4)
#define AW86927_ANACFG15_BST_PEAK_MODE_MASK	BIT(7)
#define AW86927_ANACFG15_BST_PEAK_BACK		BIT(7)
#define AW86927_ANACFG16_BST_SRC_MASK		BIT(4)
#define AW86927_ANACFG16_BST_SRC_3NS		0

#define AW8697_RAMDATA_WR_BUFFER_SIZE		2048
#define AW8697_RAMDATA_FIFO_SIZE		16
#define AW8697_MAX_BST_VO			0x1f

#define FF_EFFECT_COUNT_MAX			32

enum aw8697_chip {
	AW_CHIP_8697,
	AW_CHIP_86927,
};

enum aw8697_play_mode {
	AW8697_PLAY_STANDBY = 0,
	AW8697_PLAY_RAM = 1,
	AW8697_PLAY_RTP = 2,
	AW8697_PLAY_TRIG = 3,
	AW8697_PLAY_CONT = 4,
	AW8697_PLAY_RAM_LOOP = 5,
};

enum aw8697_activate_mode {
	AW8697_ACTIVATE_RAM = 0,
	AW8697_ACTIVATE_CONT = 1,
	AW8697_ACTIVATE_RTP = 2,
	AW8697_ACTIVATE_RAM_LOOP = 3,
};

struct aw8697_info {
	u32 mode;
	u32 f0_pre;
	u32 f0_cali_percen;
	u32 cont_drv_lvl;
	u32 cont_drv_lvl_ov;
	u32 cont_td;
	u32 cont_zc_thr;
	u32 cont_num_brk;
	u32 f0_coeff;
	u32 f0_trace_parameter[4];
	u32 bemf_config[4];
	u32 tset;
	u32 r_spare;
	u32 bstdbg[6];
	u32 trig_config[3][5];
	u32 effect_id_boundary;
	u32 effect_max;
	u32 bst_vol_default;
	u32 bst_vol_ram;
	u32 bst_vol_rtp;
	/* aw86927-only */
	u32 brk_bst_md;
	u32 cont_brk_time;
	u32 cont_tset;
	u32 cont_bemf_set;
	u32 cont_bst_brk_gain;
	u32 cont_brk_gain;
	u32 d2s_gain;
};

struct aw8697 {
	struct i2c_client *client;
	struct device *dev;
	struct gpio_desc *reset_gpio;
	struct input_dev *input_dev;
	struct mutex lock;
	struct work_struct play_work;

	enum aw8697_chip chip;

	u8 play_mode;
	u8 activate_mode;
	u8 effect_type;
	int effect_id;
	int state;
	int duration;
	u16 vmax_mv;
	u16 new_gain;
	u8 level;
	u32 ram_base_addr;
	u8 ram_init;
	int ram_verify_ok;

	struct aw8697_info info;	/* params of the detected chip */
	struct aw8697_info info_8697;	/* aw8697_vib_* params */
	struct aw8697_info info_86927;	/* aw86927_vib_* params */
};

static int aw8697_i2c_read(struct aw8697 *aw8697, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(aw8697->client, reg);
	if (ret < 0) {
		dev_err(aw8697->dev, "%s: read reg 0x%02x failed %d\n",
			__func__, reg, ret);
		return ret;
	}
	*val = ret;
	return 0;
}

static int aw8697_i2c_write(struct aw8697 *aw8697, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(aw8697->client, reg, val);
	if (ret < 0)
		dev_err(aw8697->dev, "%s: write reg 0x%02x=0x%02x failed %d\n",
			__func__, reg, val, ret);
	return ret;
}

static int aw8697_i2c_write_bits(struct aw8697 *aw8697, u8 reg, u32 mask,
				 u8 val)
{
	u8 reg_val;
	int ret;

	ret = aw8697_i2c_read(aw8697, reg, &reg_val);
	if (ret)
		return ret;
	reg_val = (reg_val & ~mask) | (val & mask);
	return aw8697_i2c_write(aw8697, reg, reg_val);
}

/*
 * Single-message register write: [reg][data...] in ONE i2c transaction.
 * i2c_smbus_write_i2c_block_data() splits this into two messages with a
 * repeated START, so the device misinterprets the first data byte as a
 * register address and the write silently fails. The mt6895 i2c runs in
 * FIFO mode only for <= fifo_size (16) total bytes, so chunk len must be
 * <= fifo_size-1 data bytes (reg byte counts too). RAMDATA auto-increments,
 * so chunked writes are equivalent to one big write.
 */
#define AW8697_I2C_WR_MAX		(AW8697_RAMDATA_FIFO_SIZE - 1)

static int aw8697_i2c_writes_single(struct aw8697 *aw8697, u8 reg,
				    const u8 *buf, int len)
{
	u8 tx[AW8697_RAMDATA_FIFO_SIZE];
	struct i2c_msg msg;
	int ret;

	msg.addr = aw8697->client->addr;
	msg.flags = 0;	/* write */
	msg.buf = tx;
	msg.len = 1 + len;
	tx[0] = reg;
	memcpy(tx + 1, buf, len);

	ret = i2c_transfer(aw8697->client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(aw8697->dev, "%s: single write reg 0x%02x len %d failed %d\n",
			__func__, reg, len, ret);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

/* Read RAM data sequentially (auto-incrementing) via single-block reads. */
static int aw8697_i2c_reads_ram(struct aw8697 *aw8697, u8 reg, u8 *buf, int len)
{
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = aw8697->client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = 1;
	msg[1].addr = aw8697->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = len;

	ret = i2c_transfer(aw8697->client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(aw8697->dev, "%s: read reg 0x%02x len %d failed %d\n",
			__func__, reg, len, ret);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

/* Read back the first n bytes of the loaded RAM firmware and compare against
 * the source buffer; the chip auto-increments RAMADDR on RAMDATA access.
 * Returns 0 on match, -ERANGE on mismatch (diagnostic for the DMA/FIFO bug).
 * Stores the result in aw8697->ram_verify_ok so the loader can disable
 * RAMINIT after verifying but still report failure. */
static int aw8697_ram_verify(struct aw8697 *aw8697, u8 ramaddr_reg,
			     u8 ramdata_reg, const u8 *data, int len,
			     int offset)
{
	u8 rbuf[AW8697_RAMDATA_FIFO_SIZE];
	u8 first_expected[32] = {0}, first_actual[32] = {0};
	int first_n = 0;
	int i, n, bad = 0;

	for (i = offset; i < len; i += AW8697_RAMDATA_FIFO_SIZE) {
		n = min(len - i, AW8697_RAMDATA_FIFO_SIZE);
		if (aw8697_i2c_reads_ram(aw8697, ramdata_reg, rbuf, n) < 0) {
			aw8697->ram_verify_ok = -EIO;
			return -EIO;
		}
		if (first_n == 0 && i == offset) {
			first_n = min(n, 32);
			memcpy(first_expected, &data[i], first_n);
			memcpy(first_actual, rbuf, first_n);
		}
		if (memcmp(rbuf, &data[i], n)) {
			bad++;
			if (bad <= 3)
				dev_err(aw8697->dev,
					"ram verify mismatch @0x%04x\n",
					aw8697->ram_base_addr + i - offset);
		}
	}
	if (bad) {
		dev_err(aw8697->dev, "RAM verify FAILED (%d/%d chunks)\n",
			bad, (len - offset + AW8697_RAMDATA_FIFO_SIZE - 1) /
			     AW8697_RAMDATA_FIFO_SIZE);
		dev_err(aw8697->dev, "expected: %*phN\nactual:   %*phN\n",
			first_n, first_expected, first_n, first_actual);
		aw8697->ram_verify_ok = -ERANGE;
		return -ERANGE;
	}
	dev_info(aw8697->dev, "RAM verify OK (%d bytes)\n", len - offset);
	aw8697->ram_verify_ok = 0;
	return 0;
}

static int aw8697_hw_reset(struct aw8697 *aw8697)
{
	gpiod_set_value_cansleep(aw8697->reset_gpio, 0);
	usleep_range(5000, 5500);
	gpiod_set_value_cansleep(aw8697->reset_gpio, 1);
	usleep_range(8000, 8500);
	return 0;
}

static void aw8697_sw_reset(struct aw8697 *aw8697)
{
	aw8697_i2c_write(aw8697, AW8697_REG_ID, 0xaa);
	usleep_range(2000, 2500);
}

static int aw8697_check_chipid(struct aw8697 *aw8697)
{
	u8 val;
	u8 idh, idl;
	int ret;
	int cnt = 0;

	while (cnt < 5) {
		ret = aw8697_hw_reset(aw8697);
		if (ret)
			return ret;
		ret = aw8697_i2c_read(aw8697, AW8697_REG_ID, &val);
		if (ret)
			return ret;
		if (val == AW8697_CHIP_ID) {
			dev_info(aw8697->dev, "AW8697 detected\n");
			aw8697->chip = AW_CHIP_8697;
			aw8697_sw_reset(aw8697);
			return 0;
		}
		/* try AW86927 (ID at 0x57/0x58) */
		ret = aw8697_i2c_read(aw8697, AW86927_REG_IDH, &idh);
		if (ret)
			return ret;
		ret = aw8697_i2c_read(aw8697, AW86927_REG_IDL, &idl);
		if (ret)
			return ret;
		if (((u16)idh << 8 | idl) == AW86927_CHIP_ID) {
			dev_info(aw8697->dev, "AW86927 detected\n");
			aw8697->chip = AW_CHIP_86927;
			aw8697_sw_reset(aw8697);
			return 0;
		}
		dev_info(aw8697->dev,
			 "unsupported chip id reg0=0x%02x reg57=0x%02x reg58=0x%02x\n",
			 val, idh, idl);
		cnt++;
		usleep_range(2000, 3000);
	}
	return -EINVAL;
}

/* ------------------------------------------------------------------ */
/* AW8697 chip path                                                    */
/* ------------------------------------------------------------------ */

static int aw8697_play_mode(struct aw8697 *aw8697, u8 mode)
{
	int ret;

	switch (mode) {
	case AW8697_PLAY_STANDBY:
		aw8697->play_mode = AW8697_PLAY_STANDBY;
		ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					    AW8697_SYSCTRL_WORK_MODE_MASK,
					    AW8697_SYSCTRL_STANDBY);
		break;
	case AW8697_PLAY_RAM:
		aw8697->play_mode = AW8697_PLAY_RAM;
		ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					    AW8697_SYSCTRL_PLAY_MODE_MASK,
					    AW8697_SYSCTRL_PLAY_MODE_RAM);
		if (ret)
			break;
		ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					    AW8697_SYSCTRL_WORK_MODE_MASK,
					    AW8697_SYSCTRL_ACTIVE);
		if (ret)
			break;
		ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					    AW8697_SYSCTRL_BST_MODE_MASK,
					    AW8697_SYSCTRL_BST_MODE_BOOST);
		break;
	case AW8697_PLAY_RAM_LOOP:
		aw8697->play_mode = AW8697_PLAY_RAM_LOOP;
		ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					    AW8697_SYSCTRL_PLAY_MODE_MASK,
					    AW8697_SYSCTRL_PLAY_MODE_RAM);
		if (ret)
			break;
		ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					    AW8697_SYSCTRL_WORK_MODE_MASK,
					    AW8697_SYSCTRL_ACTIVE);
		if (ret)
			break;
		ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					    AW8697_SYSCTRL_BST_MODE_MASK, 0);
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int aw8697_play_go(struct aw8697 *aw8697, bool en)
{
	return aw8697_i2c_write_bits(aw8697, AW8697_REG_GO, AW8697_GO_MASK,
				     en ? AW8697_GO_ENABLE : 0);
}

static int aw8697_stop_delay(struct aw8697 *aw8697)
{
	u8 val;
	int cnt = 100;

	while (cnt--) {
		if (aw8697_i2c_read(aw8697, AW8697_REG_GLB_STATE, &val))
			break;
		if ((val & 0x0f) == 0)
			return 0;
		usleep_range(2000, 2500);
	}
	return 0;
}

static int aw8697_stop(struct aw8697 *aw8697)
{
	aw8697_play_go(aw8697, false);
	aw8697_stop_delay(aw8697);
	return aw8697_play_mode(aw8697, AW8697_PLAY_STANDBY);
}

static int aw8697_start(struct aw8697 *aw8697)
{
	return aw8697_play_go(aw8697, true);
}

static int aw8697_set_wav_seq(struct aw8697 *aw8697, u8 wav, u8 seq)
{
	return aw8697_i2c_write(aw8697, AW8697_REG_WAVSEQ1 + wav, seq);
}

static int aw8697_set_wav_loop(struct aw8697 *aw8697, u8 wav, u8 loop)
{
	u8 reg;

	reg = AW8697_REG_WAVLOOP1 + wav / 2;
	if (wav % 2)
		return aw8697_i2c_write_bits(aw8697, reg,
					     AW8697_WAVLOOP_SEQNP1_MASK, loop);
	return aw8697_i2c_write_bits(aw8697, reg,
				     AW8697_WAVLOOP_SEQN_MASK, loop << 4);
}

static int aw8697_set_repeat_wav_seq(struct aw8697 *aw8697, u8 seq)
{
	aw8697_set_wav_seq(aw8697, 0, seq);
	return aw8697_set_wav_loop(aw8697, 0, AW8697_WAVLOOP_INFINITELY);
}

static int aw8697_set_bst_vol(struct aw8697 *aw8697, u8 vol)
{
	if (vol & 0xe0)
		vol = AW8697_MAX_BST_VO;
	return aw8697_i2c_write_bits(aw8697, AW8697_REG_BSTDBG4,
				     AW8697_BSTDBG4_BSTVOL_MASK, vol << 1);
}

static int aw8697_set_gain_reg(struct aw8697 *aw8697, u8 gain)
{
	return aw8697_i2c_write(aw8697, AW8697_REG_DATDBG, gain);
}

static int aw8697_effect_strength(struct aw8697 *aw8697)
{
	if (aw8697->vmax_mv >= 0x7fff)
		aw8697->level = 0x80;
	else if (aw8697->vmax_mv <= 0x3fff)
		aw8697->level = 0x1e;
	else
		aw8697->level = (aw8697->vmax_mv - 16383) / 128;
	if (aw8697->level < 0x1e)
		aw8697->level = 0x1e;
	return 0;
}

static int aw8697_play_repeat_seq(struct aw8697 *aw8697)
{
	int ret;

	ret = aw8697_play_mode(aw8697, AW8697_PLAY_RAM_LOOP);
	if (ret)
		return ret;
	return aw8697_start(aw8697);
}

static int aw8697_play_effect_seq(struct aw8697 *aw8697)
{
	int ret = 0;

	if (aw8697->effect_id > aw8697->info.effect_id_boundary)
		return 0;

	if (aw8697->activate_mode == AW8697_ACTIVATE_RAM) {
		aw8697_set_wav_seq(aw8697, 0x00, (u8)aw8697->effect_id + 1);
		aw8697_set_wav_seq(aw8697, 0x01, 0x00);
		aw8697_set_wav_loop(aw8697, 0x00, 0x00);
		ret = aw8697_play_mode(aw8697, AW8697_PLAY_RAM);
		if (ret)
			return ret;
		if (aw8697->info.bst_vol_ram <= AW8697_MAX_BST_VO)
			aw8697_set_bst_vol(aw8697,
					   (u8)aw8697->info.bst_vol_ram);
		else
			aw8697_set_bst_vol(aw8697, aw8697->level);
		aw8697_effect_strength(aw8697);
		aw8697_set_gain_reg(aw8697, aw8697->level);
		ret = aw8697_start(aw8697);
	}
	if (aw8697->activate_mode == AW8697_ACTIVATE_RAM_LOOP) {
		aw8697_set_repeat_wav_seq(aw8697,
			(u8)(aw8697->info.effect_id_boundary + 1));
		aw8697_effect_strength(aw8697);
		aw8697_set_gain_reg(aw8697, aw8697->level);
		ret = aw8697_play_repeat_seq(aw8697);
	}
	return ret;
}

static int aw8697_ram_container_update(struct aw8697 *aw8697,
				       const u8 *data, int len)
{
	int i, chunk;
	int ret;
	u32 base_addr;

	/* RAMINIT enable */
	ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				    AW8697_SYSCTRL_RAMINIT_MASK,
				    AW8697_SYSCTRL_RAMINIT_MASK);
	if (ret)
		return ret;

	base_addr = (data[2] << 8) | data[3];
	aw8697->ram_base_addr = base_addr;
	dev_info(aw8697->dev, "ram base addr 0x%04x len %d\n", base_addr, len);

	ret = aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, data[2]);
	if (ret)
		return ret;
	ret = aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, data[3]);
	if (ret)
		return ret;

	/* write the waveform data as single-message [reg][data] transactions:
	 * i2c_smbus block writes split into 2 msgs (broken), and the DMA path
	 * is broken on this SoC, so each transaction must be <= fifo_size and
	 * carry reg+data in one buffer. RAMDATA auto-increments, so chunked
	 * writes are equivalent to one big write. */
	i = 4;
	while (i < len) {
		chunk = min(len - i, AW8697_I2C_WR_MAX);
		ret = aw8697_i2c_writes_single(aw8697, AW8697_REG_RAMDATA,
					       &data[i], chunk);
		if (ret)
			return ret;
		i += chunk;
	}

	/* rewind RAMADDR and read back to prove the write landed.
	 * IMPORTANT: do this BEFORE disabling RAMINIT - the RAM is not
	 * readable once RAMINIT is off (downstream verifies in the same
	 * RAMINIT-on window). */
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, data[2]);
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, data[3]);
	ret = aw8697_ram_verify(aw8697, AW8697_REG_RAMADDRH,
				AW8697_REG_RAMDATA, data, len, 4);

	ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				    AW8697_SYSCTRL_RAMINIT_MASK, 0);
	if (ret)
		return ret;

	if (aw8697->ram_verify_ok < 0)
		return aw8697->ram_verify_ok;

	aw8697->ram_init = 1;
	return 0;
}

static int aw8697_haptic_init(struct aw8697 *aw8697)
{
	int ret;

	ret = aw8697_play_mode(aw8697, AW8697_PLAY_STANDBY);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMDBG,
				    AW8697_PWMDBG_PWM_MODE_MASK,
				    AW8697_PWMDBG_PWM_24K);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
				    AW8697_ANACTRL_LRA_SRC_MASK,
				    AW8697_ANACTRL_LRA_SRC_REG);
	if (ret)
		return ret;

	ret = aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG1,
			       (u8)aw8697->info.bstdbg[0]);
	if (ret)
		return ret;
	ret = aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG2,
			       (u8)aw8697->info.bstdbg[1]);
	if (ret)
		return ret;
	ret = aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG3,
			       (u8)aw8697->info.bstdbg[2]);
	if (ret)
		return ret;
	ret = aw8697_i2c_write(aw8697, AW8697_REG_TSET,
			       (u8)aw8697->info.tset);
	if (ret)
		return ret;
	ret = aw8697_i2c_write(aw8697, AW8697_REG_R_SPARE,
			       (u8)aw8697->info.r_spare);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_ANADBG,
				    AW8697_ANADBG_IOC_MASK,
				    AW8697_ANADBG_IOC_4P65A);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_BSTCFG,
				    AW8697_BSTCFG_PEAKCUR_MASK,
				    AW8697_BSTCFG_PEAKCUR_3P5A);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
				    AW8697_DETCTRL_PROTECT_MASK,
				    AW8697_DETCTRL_PROTECT_NO_ACTION);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMPRC,
				    AW8697_PWMPRC_PRC_MASK, 0);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
				    AW8697_BST_AUTO_AUTOSW_MASK,
				    AW8697_BST_AUTO_MANUAL_BOOST);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW8697_REG_ADCTEST,
				    AW8697_ADCTEST_VBAT_MODE_MASK,
				    AW8697_ADCTEST_VBAT_HW_COMP);
	if (ret)
		return ret;

	if (aw8697->info.f0_pre && aw8697->info.f0_coeff) {
		u32 f0_reg = 1000000000 / (aw8697->info.f0_pre *
					   aw8697->info.f0_coeff);

		ret = aw8697_i2c_write(aw8697, AW8697_REG_F_PRE_H,
				       (u8)((f0_reg >> 8) & 0xff));
		if (ret)
			return ret;
		ret = aw8697_i2c_write(aw8697, AW8697_REG_F_PRE_L,
				       (u8)(f0_reg & 0xff));
		if (ret)
			return ret;
	}

	ret = aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, 0x00);
	if (ret)
		return ret;

	return 0;
}

/* ------------------------------------------------------------------ */
/* AW86927 chip path                                                   */
/* ------------------------------------------------------------------ */

static int aw86927_raminit(struct aw8697 *aw8697, bool on)
{
	return aw8697_i2c_write_bits(aw8697, AW86927_REG_SYSCTRL3,
				     AW86927_SYSCTRL3_EN_RAMINIT_MASK,
				     on ? AW86927_SYSCTRL3_EN_RAMINIT_ON : 0);
}

static int aw86927_set_base_addr(struct aw8697 *aw8697)
{
	int ret;

	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_RTPCFG1,
				    AW86927_RTPCFG1_BASE_ADDR_H_MASK,
				    (u8)(aw8697->ram_base_addr >> 8));
	if (ret)
		return ret;
	ret = aw8697_i2c_write(aw8697, AW86927_REG_RTPCFG2,
			       (u8)(aw8697->ram_base_addr & 0xff));
	if (ret)
		return ret;
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_RAMADDRH,
				    AW86927_RAMADDRH_MASK,
				    (u8)(aw8697->ram_base_addr >> 8));
	if (ret)
		return ret;
	return aw8697_i2c_write(aw8697, AW86927_REG_RAMADDRL,
				(u8)(aw8697->ram_base_addr & 0xff));
}

static int aw86927_set_fifo_addr(struct aw8697 *aw8697)
{
	u32 base = aw8697->ram_base_addr;
	u8 ae_h, ae_l, af_h, af_l;
	int ret;

	ae_h = ((base >> 1) >> 4) & 0xf0;
	ae_l = (base >> 1) & 0xff;
	af_h = ((base - (base >> 2)) >> 8) & 0x0f;
	af_l = (base - (base >> 2)) & 0xff;

	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_RTPCFG3,
				    GENMASK(7, 4), ae_h);
	if (ret)
		return ret;
	ret = aw8697_i2c_write(aw8697, AW86927_REG_RTPCFG4, ae_l);
	if (ret)
		return ret;
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_RTPCFG3,
				    GENMASK(3, 0), af_h);
	if (ret)
		return ret;
	return aw8697_i2c_write(aw8697, AW86927_REG_RTPCFG5, af_l);
}

static int aw86927_wait_enter_standby(struct aw8697 *aw8697)
{
	u8 val;
	int cnt = 100;

	while (cnt--) {
		if (aw8697_i2c_read(aw8697, AW86927_REG_GLBRD5, &val))
			break;
		if ((val & AW86927_GLBRD5_STATE_MASK) ==
		    AW86927_GLBRD5_STATE_STANDBY)
			return 0;
		usleep_range(2000, 2500);
	}
	return 0;
}

static int aw86927_stop(struct aw8697 *aw8697)
{
	int ret;

	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_PLAYCFG4,
				    AW86927_PLAYCFG4_STOP_MASK,
				    AW86927_PLAYCFG4_STOP_ON);
	if (ret)
		return ret;
	aw86927_wait_enter_standby(aw8697);
	aw8697->play_mode = AW8697_PLAY_STANDBY;
	return 0;
}

static int aw86927_play_mode(struct aw8697 *aw8697, u8 mode)
{
	int ret;

	switch (mode) {
	case AW8697_PLAY_STANDBY:
		ret = aw86927_stop(aw8697);
		break;
	case AW8697_PLAY_RAM:
		aw8697->play_mode = AW8697_PLAY_RAM;
		ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_PLAYCFG3,
					    AW86927_PLAYCFG3_PLAY_MODE_MASK,
					    AW86927_PLAYCFG3_PLAY_MODE_RAM);
		if (ret)
			break;
		ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_PLAYCFG1,
					    AW86927_PLAYCFG1_BST_MODE_MASK,
					    AW86927_PLAYCFG1_BST_MODE);
		break;
	case AW8697_PLAY_RAM_LOOP:
		aw8697->play_mode = AW8697_PLAY_RAM_LOOP;
		ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_PLAYCFG3,
					    AW86927_PLAYCFG3_PLAY_MODE_MASK,
					    AW86927_PLAYCFG3_PLAY_MODE_RAM);
		if (ret)
			break;
		ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_PLAYCFG1,
					    AW86927_PLAYCFG1_BST_MODE_MASK, 0);
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int aw86927_play_go(struct aw8697 *aw8697, bool en)
{
	return aw8697_i2c_write_bits(aw8697, AW86927_REG_PLAYCFG4,
				     AW86927_PLAYCFG4_GO_MASK,
				     en ? AW86927_PLAYCFG4_GO_ON : 0);
}

static int aw86927_start(struct aw8697 *aw8697)
{
	return aw86927_play_go(aw8697, true);
}

static int aw86927_set_wav_seq(struct aw8697 *aw8697, u8 wav, u8 seq)
{
	return aw8697_i2c_write(aw8697, AW86927_REG_WAVCFG1 + wav, seq);
}

static int aw86927_set_wav_loop(struct aw8697 *aw8697, u8 wav, u8 loop)
{
	u8 reg = AW86927_REG_WAVCFG9 + wav / 2;

	if (wav % 2)
		return aw8697_i2c_write_bits(aw8697, reg,
					     AW86927_WAVLOOP_SEQ_EVEN_MASK,
					     loop);
	return aw8697_i2c_write_bits(aw8697, reg,
				     AW86927_WAVLOOP_SEQ_ODD_MASK, loop << 4);
}

static int aw86927_set_repeat_wav_seq(struct aw8697 *aw8697, u8 seq)
{
	aw86927_set_wav_seq(aw8697, 0, seq);
	return aw86927_set_wav_loop(aw8697, 0, AW86927_WAVLOOP_INFINITELY);
}

static int aw86927_set_bst_vol(struct aw8697 *aw8697, u8 vol)
{
	if (vol > AW86927_BST_VOUT_10P5V)
		vol = AW86927_BST_VOUT_10P5V;
	if (vol < AW86927_BST_VOUT_6V)
		vol = AW86927_BST_VOUT_6V;
	return aw8697_i2c_write_bits(aw8697, AW86927_REG_PLAYCFG1,
				     AW86927_PLAYCFG1_BST_VOUT_MASK, vol);
}

static int aw86927_set_gain_reg(struct aw8697 *aw8697, u8 gain)
{
	return aw8697_i2c_write(aw8697, AW86927_REG_PLAYCFG2, gain);
}

static int aw86927_play_effect_seq(struct aw8697 *aw8697)
{
	int ret = 0;

	if (aw8697->effect_id > aw8697->info.effect_id_boundary)
		return 0;

	if (aw8697->activate_mode == AW8697_ACTIVATE_RAM) {
		aw86927_set_wav_seq(aw8697, 0x00, (u8)aw8697->effect_id + 1);
		aw86927_set_wav_seq(aw8697, 0x01, 0x00);
		aw86927_set_wav_loop(aw8697, 0x00, 0x00);
		ret = aw86927_play_mode(aw8697, AW8697_PLAY_RAM);
		if (ret)
			return ret;
		aw86927_set_bst_vol(aw8697, (u8)aw8697->info.bst_vol_ram);
		aw8697_effect_strength(aw8697);
		aw86927_set_gain_reg(aw8697, aw8697->level);
		ret = aw86927_start(aw8697);
	}
	if (aw8697->activate_mode == AW8697_ACTIVATE_RAM_LOOP) {
		ret = aw86927_play_mode(aw8697, AW8697_PLAY_RAM_LOOP);
		if (ret)
			return ret;
		aw86927_set_repeat_wav_seq(aw8697,
			(u8)(aw8697->info.effect_id_boundary + 1));
		aw8697_effect_strength(aw8697);
		aw86927_set_gain_reg(aw8697, aw8697->level);
		ret = aw86927_start(aw8697);
	}
	return ret;
}

static int aw86927_ram_container_update(struct aw8697 *aw8697,
					const u8 *data, int len)
{
	int i, chunk;
	int ret;

	ret = aw86927_raminit(aw8697, true);
	if (ret)
		return ret;
	aw86927_stop(aw8697);

	aw8697->ram_base_addr = (data[2] << 8) | data[3];
	dev_info(aw8697->dev, "ram base addr 0x%04x len %d\n",
		 aw8697->ram_base_addr, len);

	ret = aw86927_set_base_addr(aw8697);
	if (ret)
		return ret;

	ret = aw86927_set_fifo_addr(aw8697);
	if (ret)
		return ret;

	/* single-message [reg][data] chunks only (see aw8697_ram_container_update) */
	i = 4;
	while (i < len) {
		chunk = min(len - i, AW8697_I2C_WR_MAX);
		ret = aw8697_i2c_writes_single(aw8697, AW86927_REG_RAMDATA,
					       &data[i], chunk);
		if (ret)
			return ret;
		i += chunk;
	}

	/* rewind RAMADDR and read back to prove the write landed.
	 * IMPORTANT: do this BEFORE disabling RAMINIT (downstream does the
	 * verify in the same RAMINIT-on window). */
	ret = aw86927_set_base_addr(aw8697);
	if (ret)
		return ret;
	ret = aw8697_ram_verify(aw8697, AW86927_REG_RAMADDRH,
				AW86927_REG_RAMDATA, data, len, 4);

	ret = aw86927_raminit(aw8697, false);
	if (ret)
		return ret;

	if (aw8697->ram_verify_ok < 0)
		return aw8697->ram_verify_ok;

	aw8697->ram_init = 1;
	return 0;
}

static int aw86927_haptic_init(struct aw8697 *aw8697)
{
	int ret;

	/* Unlock register */
	ret = aw8697_i2c_write(aw8697, AW86927_REG_TMCFG,
			       AW86927_TMCFG_UNLOCK);
	if (ret)
		return ret;

	ret = aw8697_i2c_write(aw8697, AW86927_REG_SYSCTRL5,
			       AW86927_SYSCTRL5_INIT_VAL);
	if (ret)
		return ret;

	/* Close boost skip */
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_ANACFG12,
				    AW86927_ANACFG12_BST_SKIP_MASK,
				    AW86927_ANACFG12_BST_SKIP_SHUTDOWN);
	if (ret)
		return ret;

	/* Adaptive ipeak */
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_ANACFG15,
				    AW86927_ANACFG15_BST_PEAK_MODE_MASK,
				    AW86927_ANACFG15_BST_PEAK_BACK);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_ANACFG16,
				    AW86927_ANACFG16_BST_SRC_MASK,
				    AW86927_ANACFG16_BST_SRC_3NS);
	if (ret)
		return ret;

	ret = aw8697_i2c_write(aw8697, AW86927_REG_PWMCFG1,
			       AW86927_PWMCFG1_INIT_VAL);
	if (ret)
		return ret;

	/* brk_bst_md */
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_CONTCFG1,
				    AW86927_CONTCFG1_BRK_BST_MD_MASK,
				    (u8)aw8697->info.brk_bst_md << 6);
	if (ret)
		return ret;

	ret = aw8697_i2c_write(aw8697, AW86927_REG_CONTCFG10,
			       (u8)aw8697->info.cont_brk_time);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_CONTCFG13,
				    AW86927_CONTCFG13_TSET_MASK,
				    (u8)aw8697->info.cont_tset << 4);
	if (ret)
		return ret;
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_CONTCFG13,
				    AW86927_CONTCFG13_BEME_SET_MASK,
				    (u8)aw8697->info.cont_bemf_set);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_CONTCFG5,
				    AW86927_CONTCFG5_BST_BRK_GAIN_MASK,
				    (u8)aw8697->info.cont_bst_brk_gain << 4);
	if (ret)
		return ret;
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_CONTCFG5,
				    AW86927_CONTCFG5_BRK_GAIN_MASK,
				    (u8)aw8697->info.cont_brk_gain);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_DETCFG2,
				    AW86927_DETCFG2_D2S_GAIN_MASK,
				    (u8)aw8697->info.d2s_gain);
	if (ret)
		return ret;

	/* Lock register */
	ret = aw8697_i2c_write(aw8697, AW86927_REG_TMCFG, AW86927_TMCFG_LOCK);
	if (ret)
		return ret;

	/* GAIN_BYPASS */
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_SYSCTRL4,
				    AW86927_SYSCTRL4_GAIN_BYPASS_MASK,
				    AW86927_SYSCTRL4_GAIN_CHANGEABLE);
	if (ret)
		return ret;

	/* PWM 24K */
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_SYSCTRL4,
				    AW86927_SYSCTRL4_WAVDAT_MODE_MASK,
				    AW86927_SYSCTRL4_WAVDAT_24K);
	if (ret)
		return ret;

	/* vbat mode: SW adjust */
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_VBATCTRL,
				    AW86927_VBATCTRL_VBAT_MODE_MASK,
				    AW86927_VBATCTRL_VBAT_MODE_SW);
	if (ret)
		return ret;

	/* default boost voltage + peak current */
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_PLAYCFG1,
				    AW86927_PLAYCFG1_BST_VOUT_MASK,
				    (u8)aw8697->info.bst_vol_default);
	if (ret)
		return ret;

	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_TMCFG,
				    GENMASK(7, 0), AW86927_TMCFG_UNLOCK);
	if (ret)
		return ret;
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_ANACFG13,
				    AW86927_ANACFG13_PEAKCUR_MASK,
				    AW86927_ANACFG13_PEAKCUR_3P45A);
	if (ret)
		return ret;
	ret = aw8697_i2c_write_bits(aw8697, AW86927_REG_TMCFG,
				    GENMASK(7, 0), AW86927_TMCFG_LOCK);
	if (ret)
		return ret;

	return 0;
}

/* ------------------------------------------------------------------ */
/* shared play/ram dispatch                                            */
/* ------------------------------------------------------------------ */

static int aw8697_ram_load(struct aw8697 *aw8697)
{
	const struct firmware *fw;
	u16 check_sum = 0;
	int i;
	int ret;

	ret = request_firmware(&fw, "aw8697_haptic.bin", aw8697->dev);
	if (ret) {
		dev_err(aw8697->dev, "failed to load aw8697_haptic.bin: %d\n",
			ret);
		return ret;
	}

	dev_info(aw8697->dev, "loaded aw8697_haptic.bin size %zu\n", fw->size);

	for (i = 2; i < fw->size; i++)
		check_sum += fw->data[i];

	if (check_sum != (u16)((fw->data[0] << 8) | fw->data[1])) {
		dev_err(aw8697->dev, "firmware checksum error 0x%04x\n",
			check_sum);
		ret = -EINVAL;
		goto out;
	}

	if (aw8697->chip == AW_CHIP_8697)
		ret = aw8697_ram_container_update(aw8697, fw->data, fw->size);
	else
		ret = aw86927_ram_container_update(aw8697, fw->data, fw->size);
out:
	release_firmware(fw);
	return ret;
}

static void aw8697_play_work(struct work_struct *work)
{
	struct aw8697 *aw8697 = container_of(work, struct aw8697, play_work);
	int ret;

	mutex_lock(&aw8697->lock);
	if (aw8697->chip == AW_CHIP_8697)
		aw8697_stop(aw8697);
	else
		aw86927_stop(aw8697);

	if (aw8697->state) {
		if (aw8697->activate_mode == AW8697_ACTIVATE_RAM ||
		    aw8697->activate_mode == AW8697_ACTIVATE_RAM_LOOP) {
			if (!aw8697->ram_init) {
				dev_warn(aw8697->dev,
					 "RAM not initialized, playing disabled\n");
				mutex_unlock(&aw8697->lock);
				return;
			}
			if (aw8697->chip == AW_CHIP_8697)
				ret = aw8697_play_effect_seq(aw8697);
			else
				ret = aw86927_play_effect_seq(aw8697);
			if (ret)
				dev_err(aw8697->dev, "play effect failed %d\n",
					ret);
		}
	}
	mutex_unlock(&aw8697->lock);
}

static int aw8697_upload_effect(struct input_dev *dev, struct ff_effect *effect,
				struct ff_effect *old)
{
	struct aw8697 *aw8697 = input_get_drvdata(dev);
	s16 data[3];
	int ret = 0;

	aw8697->effect_type = effect->type;

	mutex_lock(&aw8697->lock);

	if (aw8697->effect_type == FF_CONSTANT ||
	    aw8697->effect_type == FF_RUMBLE) {
		/*
		 * Both FF_CONSTANT and FF_RUMBLE map to continuous RAM-loop
		 * playback.  FF_RUMBLE carries strong_magnitude / weak_magnitude
		 * (both 0-65535); we take the stronger of the two.
		 */
		aw8697->duration = effect->replay.length;
		aw8697->activate_mode = AW8697_ACTIVATE_RAM_LOOP;
		aw8697->effect_id = aw8697->info.effect_id_boundary;
		if (aw8697->effect_type == FF_RUMBLE) {
			u16 strong = effect->u.rumble.strong_magnitude;
			u16 weak   = effect->u.rumble.weak_magnitude;
			aw8697->vmax_mv = max(strong, weak);
		} else {
			aw8697->vmax_mv = effect->u.constant.level;
		}
	} else if (aw8697->effect_type == FF_PERIODIC) {
		if (effect->u.periodic.waveform != FF_CUSTOM) {
			ret = -EINVAL;
			goto out;
		}
		if (effect->u.periodic.custom_len != sizeof(data)) {
			ret = -EINVAL;
			goto out;
		}
		if (copy_from_user(data, effect->u.periodic.custom_data,
				   sizeof(data))) {
			ret = -EFAULT;
			goto out;
		}
		aw8697->effect_id = data[0];
		aw8697->vmax_mv = effect->u.periodic.magnitude;
		if (aw8697->effect_id < 0 ||
		    aw8697->effect_id > aw8697->info.effect_max) {
			ret = -EINVAL;
			goto out;
		}
		if (aw8697->effect_id < aw8697->info.effect_id_boundary)
			aw8697->activate_mode = AW8697_ACTIVATE_RAM;
		else
			aw8697->activate_mode = AW8697_ACTIVATE_RTP;
	} else {
		ret = -EINVAL;
	}
out:
	mutex_unlock(&aw8697->lock);
	return ret;
}

static int aw8697_playback(struct input_dev *dev, int effect_id, int val)
{
	struct aw8697 *aw8697 = input_get_drvdata(dev);

	if (aw8697->activate_mode == AW8697_ACTIVATE_RTP)
		return 0;	/* RTP not supported in this port */

	aw8697->state = val > 0;
	schedule_work(&aw8697->play_work);
	return 0;
}

static int aw8697_erase(struct input_dev *dev, int effect_id)
{
	struct aw8697 *aw8697 = input_get_drvdata(dev);

	aw8697->effect_type = 0;
	aw8697->duration = 0;
	return 0;
}

static void aw8697_set_gain(struct input_dev *dev, u16 gain)
{
	struct aw8697 *aw8697 = input_get_drvdata(dev);

	aw8697->new_gain = gain;
}

static void aw8697_close(struct input_dev *dev)
{
	struct aw8697 *aw8697 = input_get_drvdata(dev);

	cancel_work_sync(&aw8697->play_work);
	aw8697->state = 0;
	if (aw8697->chip == AW_CHIP_8697)
		aw8697_stop(aw8697);
	else
		aw86927_stop(aw8697);
}

static int aw8697_parse_dt(struct aw8697 *aw8697, struct device_node *np)
{
	struct aw8697_info *a97 = &aw8697->info_8697;
	struct aw8697_info *a927 = &aw8697->info_86927;

	/* AW8697 params */
	of_property_read_u32(np, "aw8697_vib_mode", &a97->mode);
	of_property_read_u32(np, "aw8697_vib_f0_pre", &a97->f0_pre);
	of_property_read_u32(np, "aw8697_vib_f0_cali_percen",
			     &a97->f0_cali_percen);
	of_property_read_u32(np, "aw8697_vib_cont_drv_lev", &a97->cont_drv_lvl);
	of_property_read_u32(np, "aw8697_vib_cont_drv_lvl_ov",
			     &a97->cont_drv_lvl_ov);
	of_property_read_u32(np, "aw8697_vib_cont_td", &a97->cont_td);
	of_property_read_u32(np, "aw8697_vib_cont_zc_thr", &a97->cont_zc_thr);
	of_property_read_u32(np, "aw8697_vib_cont_num_brk", &a97->cont_num_brk);
	of_property_read_u32(np, "aw8697_vib_f0_coeff", &a97->f0_coeff);
	of_property_read_u32_array(np, "aw8697_vib_f0_trace_parameter",
				   a97->f0_trace_parameter, 4);
	of_property_read_u32_array(np, "aw8697_vib_bemf_config",
				   a97->bemf_config, 4);
	of_property_read_u32(np, "aw8697_vib_tset", &a97->tset);
	of_property_read_u32(np, "aw8697_vib_r_spare", &a97->r_spare);
	of_property_read_u32_array(np, "aw8697_vib_bstdbg", a97->bstdbg, 6);
	of_property_read_u32_array(np, "aw8697_vib_trig_config",
				   (u32 *)a97->trig_config, 15);
	of_property_read_u32(np, "aw8697_vib_bst_vol_default",
			     &a97->bst_vol_default);
	of_property_read_u32(np, "aw8697_vib_bst_vol_ram", &a97->bst_vol_ram);
	of_property_read_u32(np, "aw8697_vib_bst_vol_rtp", &a97->bst_vol_rtp);

	/* AW86927 params (dual-BOM board) */
	of_property_read_u32(np, "aw86927_vib_mode", &a927->mode);
	of_property_read_u32(np, "aw86927_vib_f0_pre", &a927->f0_pre);
	of_property_read_u32(np, "aw86927_vib_f0_cali_percen",
			     &a927->f0_cali_percen);
	of_property_read_u32(np, "aw86927_vib_cont_drv1_lvl",
			     &a927->cont_drv_lvl);
	of_property_read_u32(np, "aw86927_vib_cont_drv2_lvl",
			     &a927->cont_drv_lvl_ov);
	of_property_read_u32(np, "aw86927_vib_f0_coeff", &a927->f0_coeff);
	of_property_read_u32_array(np, "aw86927_vib_f0_trace_parameter",
				   a927->f0_trace_parameter, 4);
	of_property_read_u32_array(np, "aw86927_vib_bemf_config",
				   a927->bemf_config, 4);
	of_property_read_u32_array(np, "aw86927_vib_trig_config",
				   (u32 *)a927->trig_config, 15);
	of_property_read_u32(np, "aw86927_vib_bst_vol_default",
			     &a927->bst_vol_default);
	of_property_read_u32(np, "aw86927_vib_bst_vol_ram", &a927->bst_vol_ram);
	of_property_read_u32(np, "aw86927_vib_bst_vol_rtp", &a927->bst_vol_rtp);
	of_property_read_u32(np, "aw86927_vib_brk_bst_md", &a927->brk_bst_md);
	of_property_read_u32(np, "aw86927_vib_cont_brk_time",
			     &a927->cont_brk_time);
	of_property_read_u32(np, "aw86927_vib_cont_tset", &a927->cont_tset);
	of_property_read_u32(np, "aw86927_vib_cont_bemf_set",
			     &a927->cont_bemf_set);
	of_property_read_u32(np, "aw86927_vib_cont_bst_brk_gain",
			     &a927->cont_bst_brk_gain);
	of_property_read_u32(np, "aw86927_vib_cont_brk_gain",
			     &a927->cont_brk_gain);
	of_property_read_u32(np, "aw86927_vib_d2s_gain", &a927->d2s_gain);

	/* common */
	of_property_read_u32(np, "vib_effect_id_boundary",
			     &a97->effect_id_boundary);
	of_property_read_u32(np, "vib_effect_max", &a97->effect_max);
	a927->effect_id_boundary = a97->effect_id_boundary;
	a927->effect_max = a97->effect_max;

	dev_info(aw8697->dev,
		 "aw8697 bst_ram=0x%x aw86927 bst_ram=0x%x boundary %u max %u\n",
		 a97->bst_vol_ram, a927->bst_vol_ram,
		 a97->effect_id_boundary, a97->effect_max);
	return 0;
}

static int aw8697_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct aw8697 *aw8697;
	struct input_dev *input_dev;
	struct ff_device *ff;
	int ret;

	aw8697 = devm_kzalloc(dev, sizeof(*aw8697), GFP_KERNEL);
	if (!aw8697)
		return -ENOMEM;

	aw8697->client = client;
	aw8697->dev = dev;
	mutex_init(&aw8697->lock);
	INIT_WORK(&aw8697->play_work, aw8697_play_work);
	i2c_set_clientdata(client, aw8697);

	aw8697->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw8697->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(aw8697->reset_gpio),
				     "failed to get reset gpio\n");

	ret = aw8697_parse_dt(aw8697, dev->of_node);
	if (ret)
		return ret;

	ret = aw8697_check_chipid(aw8697);
	if (ret)
		return dev_err_probe(dev, ret, "AW8697/AW86927 not found\n");

	/* select the DT param table for the detected chip */
	if (aw8697->chip == AW_CHIP_8697)
		aw8697->info = aw8697->info_8697;
	else
		aw8697->info = aw8697->info_86927;
	dev_info(dev, "chip params bst_ram=0x%x bst_default=0x%x\n",
		 aw8697->info.bst_vol_ram, aw8697->info.bst_vol_default);

	if (aw8697->chip == AW_CHIP_8697)
		ret = aw8697_haptic_init(aw8697);
	else
		ret = aw86927_haptic_init(aw8697);
	if (ret)
		return dev_err_probe(dev, ret, "haptic init failed\n");

	/* load the firmware (predefined waveforms) into SRAM */
	ret = aw8697_ram_load(aw8697);
	if (ret)
		dev_warn(dev, "RAM load deferred/failed (%d), effects disabled\n",
			 ret);

	input_dev = devm_input_allocate_device(dev);
	if (!input_dev)
		return -ENOMEM;

	input_dev->name = "awinic_haptic";
	input_dev->close = aw8697_close;
	input_set_drvdata(input_dev, aw8697);
	aw8697->input_dev = input_dev;

	input_set_capability(input_dev, EV_FF, FF_CONSTANT);
	input_set_capability(input_dev, EV_FF, FF_RUMBLE);
	input_set_capability(input_dev, EV_FF, FF_GAIN);
	if (aw8697->info.effect_id_boundary) {
		input_set_capability(input_dev, EV_FF, FF_PERIODIC);
		input_set_capability(input_dev, EV_FF, FF_CUSTOM);
	}

	ret = input_ff_create(input_dev, FF_EFFECT_COUNT_MAX);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create ff\n");

	ff = input_dev->ff;
	ff->upload = aw8697_upload_effect;
	ff->playback = aw8697_playback;
	ff->erase = aw8697_erase;
	ff->set_gain = aw8697_set_gain;

	ret = input_register_device(input_dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input\n");

	dev_info(dev, "AW8697/AW86927 haptic probed\n");
	return 0;
}

static const struct of_device_id aw8697_of_match[] = {
	{ .compatible = "awinic,aw8697" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, aw8697_of_match);

static struct i2c_driver aw8697_driver = {
	.driver = {
		.name = "aw8697-haptics",
		.of_match_table = aw8697_of_match,
	},
	.probe = aw8697_probe,
};
module_i2c_driver(aw8697_driver);

MODULE_AUTHOR("AWINIC Technology CO., LTD");
MODULE_DESCRIPTION("AW8697/AW86927 LRA Haptic Driver");
MODULE_LICENSE("GPL v2");
MODULE_FIRMWARE("aw8697_haptic.bin");
