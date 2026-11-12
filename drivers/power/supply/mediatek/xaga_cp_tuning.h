/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xaga / xagapro vendor charging policy (mainline port)
 *
 * Ports the downstream step_jeita_charge.c policy tables and state machine:
 * JEITA FCC/FV, VBAT step-charge (low/high cycle), FFC gating, software CV
 * and the TUNE taper.  It only computes the FCC/FV/taper targets; the CP
 * manager still owns the PPS/charge-pump loops.
 */
#ifndef _XAGA_CP_TUNING_H
#define _XAGA_CP_TUNING_H

#include <linux/device.h>
#include <linux/of.h>
#include <linux/types.h>

#define XAGA_CP_STEPS		6
#define XAGA_CP_THERMAL_LEVELS	16

/* [low, high, value] used by the step-charge and JEITA FV tables */
struct xaga_cp_step {
	int low;
	int high;
	int value;
};

/* [tmin, tmax, vbat_threshold, fcc_low, fcc_high] used by JEITA FCC */
struct xaga_cp_jeita {
	int low;
	int high;
	int vbat_threshold;
	int fcc_low;
	int fcc_high;
};

struct xaga_cp_tuning {
	bool xagapro;
	bool skip_pd_auth;
	bool force_low_cycle;	/* use the <100-cycle step table */

	/* tables, in vendor units: mV, mA, 0.1 degC */
	struct xaga_cp_jeita jeita_fcc[XAGA_CP_STEPS];
	struct xaga_cp_step jeita_fv[XAGA_CP_STEPS];
	struct xaga_cp_step step_low[XAGA_CP_STEPS];
	struct xaga_cp_step step_high[XAGA_CP_STEPS];
	int thermal[XAGA_CP_THERMAL_LEVELS];

	int fv;
	int fv_ffc;
	int iterm;
	int iterm_ffc;
	int iterm_ffc_warm;
	int ffc_low_tbat;
	int ffc_medium_tbat;
	int ffc_high_tbat;
	int ffc_high_soc;
	int step_fallback_hyst;
	int step_forward_hyst;
	int jeita_fallback_hyst;
	int jeita_forward_hyst;
	int cv_vbat;
	int cv_vbat_ffc;
	int cv_ibat;
	int high_soc;
	int low_fcc;
	int high_vbat;
	int max_fcc;

	/* state */
	int jeita_index[2];
	int step_index[2];
	int taper_count;
	int sw_cv;
	int sw_cv_count;
	int entry_soc;
	bool jeita_vbat_low;
	bool ffc_enable;
	bool recharge;
};

struct xaga_cp_inputs {
	int vbat_mv;
	int ichg_ma;		/* charging current, positive */
	int tbat_dc;		/* 0.1 degC */
	int soc;
	int cycle_count;
	int thermal_level;
	bool pd_verified;
	bool gauge_authentic;
	bool charge_full;
};

struct xaga_cp_policy {
	int target_fcc_ma;
	int fv_mv;
	int iterm_ma;
	int step_fcc_ma;
	int jeita_fcc_ma;
	int jeita_fv_mv;
	int thermal_fcc_ma;
	int ffc_fcc_ma;
	int jeita_index;
	int step_index;
	int sw_cv_mv;
	bool ffc;
	bool taper;
};

void xaga_cp_tuning_init(struct xaga_cp_tuning *t, bool xagapro);
int xaga_cp_tuning_parse(struct device *dev, struct device_node *np,
			 struct xaga_cp_tuning *t);
void xaga_cp_tuning_reset(struct xaga_cp_tuning *t);
void xaga_cp_tuning_entry(struct xaga_cp_tuning *t, int soc,
			  int vbat_mv, int tbat_dc);
void xaga_cp_policy_update(struct xaga_cp_tuning *t,
			   const struct xaga_cp_inputs *in,
			   struct xaga_cp_policy *out);

#endif /* _XAGA_CP_TUNING_H */
