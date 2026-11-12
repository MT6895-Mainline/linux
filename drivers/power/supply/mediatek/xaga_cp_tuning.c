// SPDX-License-Identifier: GPL-2.0-only
/*
 * xaga / xagapro vendor charging policy (mainline port)
 *
 * Direct port of the downstream step_jeita_charge.c tables and state
 * machine.  All values are the vendor xaga/xagapro values from
 * arch/arm64/boot/dts/vendor/mediatek/xaga-mt6895.dtsi:215-322.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "xaga_cp_tuning.h"

#define XAGA_CP_AUTH_FCC_MA	5800
#define XAGA_CP_FFC_UNAUTH_MA	2000
#define XAGA_CP_FFC_LEVEL_MAX	13
#define XAGA_CP_CV_HYST_MV	50
#define XAGA_CP_TAPER_TICKS	3

static bool xaga_cp_between(int low, int high, int value)
{
	return (low <= value && value <= high) ||
	       (high <= value && value <= low);
}

static void xaga_cp_get_step_index(const struct xaga_cp_step *cfg, int n,
				   int fallback_hyst, int forward_hyst,
				   int value, int *index, bool ignore_hyst)
{
	int new_index = 0, i;

	if (value < cfg[0].low) {
		index[0] = index[1] = 0;
		return;
	}
	if (value > cfg[n - 1].high)
		new_index = n - 1;
	else
		for (i = 0; i < n; i++)
			if (xaga_cp_between(cfg[i].low, cfg[i].high, value)) {
				new_index = i;
				break;
			}

	if (!ignore_hyst) {
		if (new_index > index[0]) {
			if (value < cfg[new_index].low + forward_hyst)
				new_index = index[0];
		} else if (new_index < index[0]) {
			if (value > cfg[new_index].high - fallback_hyst)
				new_index = index[0];
		}
		index[1] = index[0];
		index[0] = new_index;
	} else {
		index[0] = index[1] = new_index;
	}
}

static const struct xaga_cp_jeita xaga_jeita_fcc[XAGA_CP_STEPS] = {
	{ -100,   0, 4200,  1000,   747 },
	{    1,  50, 4200,  2390,  2390 },
	{   51, 100, 4200,  3386,  3386 },
	{  101, 150, 4200,  5826,  5826 },
	{  151, 480, 4200, 12400, 12400 },
	{  481, 600, 4200,  2490,  2490 },
};

static const struct xaga_cp_step xaga_jeita_fv[XAGA_CP_STEPS] = {
	{ -100,   0, 4440 },
	{    1,  50, 4450 },
	{   51, 100, 4450 },
	{  101, 150, 4450 },
	{  151, 480, 4480 },
	{  481, 600, 4100 },
};

static const struct xaga_cp_step xaga_step_low[XAGA_CP_STEPS] = {
	{ 3000, 3595, 12310 },
	{ 3596, 4090, 12310 },
	{ 4091, 4190, 12310 },
	{ 4191, 4290,  8875 },
	{ 4291, 4390,  7380 },
	{ 4391, 4480,  5786 },
};

static const struct xaga_cp_step xaga_step_high[XAGA_CP_STEPS] = {
	{ 3000, 3595, 12310 },
	{ 3596, 3850, 12310 },
	{ 3851, 4150, 12310 },
	{ 4151, 4300,  8875 },
	{ 4301, 4400,  7380 },
	{ 4401, 4480,  5786 },
};

static const int xaga_thermal[XAGA_CP_THERMAL_LEVELS] = {
	12400, 10000, 9000, 8000, 7400, 6100, 5600, 5000,
	 4400,  3500, 3000, 2400, 2000, 1500, 1000,  500,
};

static const struct xaga_cp_jeita xagapro_jeita_fcc[XAGA_CP_STEPS] = {
	{ -100,   0, 4150,  2150,   860 },
	{    1,  50, 4150,  3010,  3010 },
	{   51, 100, 4150,  5160,  5160 },
	{  101, 150, 4150,  8600,  8600 },
	{  151, 480, 4150, 21500, 21500 },
	{  481, 600, 4150,  2150,  2150 },
};

static const struct xaga_cp_step xagapro_jeita_fv[XAGA_CP_STEPS] = {
	{ -100,   0, 4440 },
	{    1,  50, 4440 },
	{   51, 100, 4446 },
	{  101, 150, 4446 },
	{  151, 480, 4500 },
	{  481, 600, 4100 },
};

static const struct xaga_cp_step xagapro_step_low[XAGA_CP_STEPS] = {
	{ 3000, 3595, 21430 },
	{ 3596, 4090, 21430 },
	{ 4091, 4140, 18910 },
	{ 4141, 4240, 15930 },
	{ 4241, 4390, 12360 },
	{ 4391, 4500,  8460 },
};

static const struct xaga_cp_step xagapro_step_high[XAGA_CP_STEPS] = {
	{ 3000, 3595, 21430 },
	{ 3596, 4080, 21430 },
	{ 4081, 4120, 18910 },
	{ 4121, 4250, 15930 },
	{ 4251, 4400, 12360 },
	{ 4401, 4500,  8460 },
};

static const int xagapro_thermal[XAGA_CP_THERMAL_LEVELS] = {
	21500, 17500, 16000, 12000, 10500, 10300, 10000, 8000,
	 6800,  6000,  5000,  4000,  3000,  2000,  1000,  500,
};

void xaga_cp_tuning_init(struct xaga_cp_tuning *t, bool xagapro)
{
	memset(t, 0, sizeof(*t));
	t->xagapro = xagapro;
	t->skip_pd_auth = true;		/* no batterysecret on mainline */
	t->force_low_cycle = false;

	if (xagapro) {
		memcpy(t->jeita_fcc, xagapro_jeita_fcc, sizeof(t->jeita_fcc));
		memcpy(t->jeita_fv, xagapro_jeita_fv, sizeof(t->jeita_fv));
		memcpy(t->step_low, xagapro_step_low, sizeof(t->step_low));
		memcpy(t->step_high, xagapro_step_high, sizeof(t->step_high));
		memcpy(t->thermal, xagapro_thermal, sizeof(t->thermal));
		t->fv = 4445;
		t->fv_ffc = 4495;
		t->iterm = 200;
		t->iterm_ffc = 2160;
		t->iterm_ffc_warm = 3370;
		t->ffc_low_tbat = 150;
		t->ffc_medium_tbat = 350;
		t->ffc_high_tbat = 480;
		t->ffc_high_soc = 95;
		t->step_fallback_hyst = 25;
		t->step_forward_hyst = 5;
		t->jeita_fallback_hyst = 5;
		t->jeita_forward_hyst = 5;
		t->cv_vbat = 4440;
		t->cv_vbat_ffc = 4490;
		t->cv_ibat = 2500;
		t->high_soc = 95;
		t->low_fcc = 2000;
		t->high_vbat = 4400;
		t->max_fcc = 21500;
	} else {
		memcpy(t->jeita_fcc, xaga_jeita_fcc, sizeof(t->jeita_fcc));
		memcpy(t->jeita_fv, xaga_jeita_fv, sizeof(t->jeita_fv));
		memcpy(t->step_low, xaga_step_low, sizeof(t->step_low));
		memcpy(t->step_high, xaga_step_high, sizeof(t->step_high));
		memcpy(t->thermal, xaga_thermal, sizeof(t->thermal));
		t->fv = 4450;
		t->fv_ffc = 4475;
		t->iterm = 200;
		t->iterm_ffc = 800;
		t->iterm_ffc_warm = 900;
		t->ffc_low_tbat = 150;
		t->ffc_medium_tbat = 350;
		t->ffc_high_tbat = 480;
		t->ffc_high_soc = 95;
		t->step_fallback_hyst = 25;
		t->step_forward_hyst = 5;
		t->jeita_fallback_hyst = 5;
		t->jeita_forward_hyst = 5;
		t->cv_vbat = 4440;
		t->cv_vbat_ffc = 4470;
		t->cv_ibat = 2000;
		t->high_soc = 95;
		t->low_fcc = 2000;
		t->high_vbat = 4400;
		t->max_fcc = 12400;
	}

	t->entry_soc = -1;
	t->jeita_vbat_low = true;
	xaga_cp_tuning_reset(t);
}

int xaga_cp_tuning_parse(struct device *dev, struct device_node *np,
			 struct xaga_cp_tuning *t)
{
	u32 val;

	if (!np)
		return 0;

	if (!of_property_read_u32(np, "xaga,skip-pd-auth", &val))
		t->skip_pd_auth = !!val;
	if (!of_property_read_u32(np, "xaga,force-low-cycle", &val))
		t->force_low_cycle = !!val;

	/*
	 * The vendor tables are compiled in above.  A board can override the
	 * two values that depend on the gauge/cell age without carrying the
	 * whole table:
	 *   xaga,cv-ibat-ma = <800>;
	 *   xaga,fv-ffc-microvolt = <4475000>;
	 */
	if (!of_property_read_u32(np, "xaga,cv-ibat-ma", &val))
		t->cv_ibat = val;
	if (!of_property_read_u32(np, "xaga,fv-ffc-microvolt", &val))
		t->fv_ffc = val / 1000;
	if (!of_property_read_u32(np, "xaga,fv-microvolt", &val))
		t->fv = val / 1000;
	if (!of_property_read_u32(np, "xaga,max-fcc-microamp", &val))
		t->max_fcc = val / 1000;

	return 0;
}

void xaga_cp_tuning_reset(struct xaga_cp_tuning *t)
{
	t->jeita_index[0] = t->jeita_index[1] = 0;
	t->step_index[0] = t->step_index[1] = 0;
	t->taper_count = 0;
	t->sw_cv = 0;
	t->sw_cv_count = 0;
	t->ffc_enable = false;
	t->recharge = false;
}

void xaga_cp_tuning_entry(struct xaga_cp_tuning *t, int soc,
			  int vbat_mv, int tbat_dc)
{
	t->entry_soc = soc;
	t->taper_count = 0;
	t->ffc_enable = false;
	/* seed both tables from the real VBAT/temperature (ignore hyst) */
	xaga_cp_get_step_index(t->step_low, XAGA_CP_STEPS,
			       t->step_fallback_hyst, t->step_forward_hyst,
			       vbat_mv, t->step_index, true);
	xaga_cp_get_step_index(t->jeita_fv, XAGA_CP_STEPS,
			       t->jeita_fallback_hyst, t->jeita_forward_hyst,
			       tbat_dc, t->jeita_index, true);
}

void xaga_cp_policy_update(struct xaga_cp_tuning *t,
			   const struct xaga_cp_inputs *in,
			   struct xaga_cp_policy *out)
{
	const struct xaga_cp_step *step;
	int ichg = max(in->ichg_ma, 0);
	int idx, jeita_fcc, step_fcc, fv, iterm, ffc_fcc, thermal_fcc;
	int target, auth_cap = INT_MAX;

	memset(out, 0, sizeof(*out));

	if (t->entry_soc < 0)
		t->entry_soc = in->soc;

	/* ---- step-charge (VBAT table, cycle-count dependent) ---- */
	step = (t->force_low_cycle || in->cycle_count <= 100) ?
		t->step_low : t->step_high;
	xaga_cp_get_step_index(step, XAGA_CP_STEPS, t->step_fallback_hyst,
			       t->step_forward_hyst, in->vbat_mv,
			       t->step_index, false);
	idx = t->step_index[0];
	step_fcc = step[idx].value + (idx == XAGA_CP_STEPS - 1 ? 100 : 0);

	/* vendor sw_cv: hold the new step until the current has fallen to it */
	if (t->step_index[0] > t->step_index[1] &&
	    step[idx].value != step[t->step_index[1]].value) {
		t->sw_cv = step[idx].low + t->step_forward_hyst;
		t->sw_cv_count = 0;
	}
	if (t->sw_cv) {
		if (ichg <= step_fcc) {
			if (++t->sw_cv_count >= 3) {
				t->sw_cv = 0;
				t->sw_cv_count = 0;
			}
		} else {
			t->sw_cv_count = 0;
		}
	}

	/* ---- JEITA FV + FCC ---- */
	xaga_cp_get_step_index(t->jeita_fv, XAGA_CP_STEPS,
				t->jeita_fallback_hyst,
				t->jeita_forward_hyst, in->tbat_dc,
				t->jeita_index, false);
	idx = t->jeita_index[0];
	{
		const struct xaga_cp_jeita *j = &t->jeita_fcc[idx];

		if (t->jeita_vbat_low) {
			if (in->vbat_mv < j->vbat_threshold + XAGA_CP_CV_HYST_MV) {
				jeita_fcc = j->fcc_low;
			} else {
				jeita_fcc = j->fcc_high;
				t->jeita_vbat_low = false;
			}
		} else {
			if (in->vbat_mv < j->vbat_threshold - XAGA_CP_CV_HYST_MV) {
				jeita_fcc = j->fcc_low;
				t->jeita_vbat_low = true;
			} else {
				jeita_fcc = j->fcc_high;
			}
		}
	}

	/* ---- FFC gating ---- */
	t->ffc_enable = !t->recharge && !in->charge_full &&
			t->entry_soc <= t->ffc_high_soc &&
			xaga_cp_between(t->ffc_low_tbat, t->ffc_high_tbat,
					in->tbat_dc) &&
			(in->pd_verified || t->skip_pd_auth) &&
			in->thermal_level <= XAGA_CP_FFC_LEVEL_MAX;

	fv = min(t->jeita_fv[idx].value,
		 t->ffc_enable ? t->fv_ffc : t->fv);

	if (t->ffc_enable) {
		iterm = in->tbat_dc >= t->ffc_medium_tbat ?
			t->iterm_ffc_warm : t->iterm_ffc;
		ffc_fcc = (in->gauge_authentic || t->skip_pd_auth) ?
			t->max_fcc : XAGA_CP_FFC_UNAUTH_MA;
	} else {
		iterm = in->tbat_dc >= t->ffc_high_tbat ?
			t->iterm_ffc_warm : t->iterm;
		ffc_fcc = t->max_fcc;
	}

	/* ---- thermal table (level from the board's 48.5 C policy) ---- */
	thermal_fcc = t->thermal[clamp_t(int, in->thermal_level, 0,
					 XAGA_CP_THERMAL_LEVELS - 1)];

	/* ---- effective FCC ---- */
	target = min(min(step_fcc, jeita_fcc), min(thermal_fcc, ffc_fcc));
	target = min(target, t->max_fcc);
	if (!in->pd_verified && !t->skip_pd_auth)
		auth_cap = XAGA_CP_AUTH_FCC_MA;
	target = min(target, auth_cap);

	/* ---- taper / CV hand-off ---- */
	{
		int cv_vbat = t->ffc_enable ? t->cv_vbat_ffc : t->cv_vbat;

		if (ichg < t->cv_ibat && in->vbat_mv > cv_vbat)
			t->taper_count++;
		else
			t->taper_count = 0;
	}

	out->target_fcc_ma = max(target, 0);
	out->fv_mv = fv;
	out->iterm_ma = iterm;
	out->step_fcc_ma = step_fcc;
	out->jeita_fcc_ma = jeita_fcc;
	out->jeita_fv_mv = t->jeita_fv[idx].value;
	out->thermal_fcc_ma = thermal_fcc;
	out->ffc_fcc_ma = ffc_fcc;
	out->jeita_index = t->jeita_index[0];
	out->step_index = t->step_index[0];
	out->sw_cv_mv = t->sw_cv;
	out->ffc = t->ffc_enable;
	out->taper = t->taper_count > XAGA_CP_TAPER_TICKS;
}
