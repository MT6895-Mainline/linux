/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Minimal fmeter interface for the SCP port.
 *
 * WORKAROUND: the stock tree provides a real fmeter
 * driver (drivers/clk/mediatek/clk-fmeter.c) that we have not ported. The only
 * SCP-side caller is scp_dvfs.c's mt_get_fmeter_freq(), which is only reached
 * when the scp_dvfs DT node sets "ccf-fmeter-support" — our DT does not, so
 * scp_dvfs takes its built-in legacy ULPOSC fmeter path instead. This stub
 * satisfies the include/link; if it ever gets called it returns 0 and the
 * caller logs a notice (scp_dvfs.c:1446). Replace by porting the real fmeter
 * driver if SCP DVFS frequency readout is ever needed.
 */
#ifndef __CLK_FMETER_H
#define __CLK_FMETER_H

#define FM_SYS(_id)		((_id & (0xFF00)) >> 8)
#define FM_ID(_id)		(_id & (0xFF))

enum FMETER_TYPE {
	FT_NULL,
	ABIST,
	CKGEN,
	ABIST_2,
	SUBSYS,
	VLPCK,
};

enum FMETER_ID {
	FID_NULL = -1,
	FID_DISP_PWM = 0,
	FID_ULPOSC1,
	FID_ULPOSC2,
	FID_NUM,
};

static inline unsigned int mt_get_fmeter_freq(unsigned int id,
					      enum FMETER_TYPE type)
{
	return 0;
}

static inline int mt_get_fmeter_id(enum FMETER_ID fid)
{
	return -1;
}

#endif /* __CLK_FMETER_H */
