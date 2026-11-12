// SPDX-License-Identifier: GPL-2.0-only
/*
 * xaga / xagapro charge-pump manager (mainline)
 *
 * Bridges the mainline TCPM PPS/APDO power-supply interface to the
 * SC8551 (xaga) charge pumps, in place of the downstream MediaTek
 * adapter_class / pd_cp_manager stack.
 *
 * The charge pumps are always kept off until a negotiated PPS contract
 * exists (or "force" is set for a controlled fixed-PDO test).  When a
 * pump is switched on the MT6375 buck is stopped first so the two
 * chargers never fight over the same VBUS.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/usb/tcpm.h>

#include "charger_class.h"
#include "xaga_cp_tuning.h"

/* cp_set_mode() values (SC8561 4:1 / SC8551 & SC8561 2:1) */
#define XAGA_CP_MODE_4_1	0
#define XAGA_CP_MODE_2_1	1
/* MT6375 AICR to restore for the direct-charge path (uA) */
#define XAGA_MT6375_AICR	3000000

/* closed-loop regulation */
#define XAGA_CPM_RES		2	/* 2:1 charge pump */
#define XAGA_CPM_STEP_MV	20
#define XAGA_CPM_REG_MS		1000
/* give a source that just dropped PPS time to recover before retrying */
#define XAGA_CPM_PPS_RETRY_MS	5000
#define XAGA_CPM_RAMP_UA	300000
#define XAGA_CPM_IBUS_GAP_MA	400
/* a 4:1 divider needs ~4*(VBAT+headroom); downstream uses 18..22 V APDOs */
#define XAGA_CPM_4_1_MIN_VBUS_UV	18000000
/*
 * SC8561 input window above ratio*VBAT.  The part refuses to switch if the
 * headroom is too large (VBUS_ERRORHI), so xagapro's request is held within
 * ~ratio*VBAT + this gap (downstream uses a 600..850 mV window).
 */
#define XAGA_CPM_VBUS_GAP_MV		1800	/* initial window */
#define XAGA_CPM_VBUS_GAP_MIN_MV	300
/*
 * A pump that latches off (VBUS_ERRORHI) leaves its enable bit set, so the
 * "!enabled" retry never fires.  If one is on but drawing almost nothing
 * while we are still asking for real current, shrink the operating window
 * and cycle the pump so it restarts lower; creep the window back up when it
 * runs clean, so the loop converges on the largest non-latching headroom.
 */
#define XAGA_CPM_STALL_IBUS_UA		150000
#define XAGA_CPM_STALL_TICKS		3
#define XAGA_CPM_STALL_BACKOFF_MV	300
/*
 * The SC8551's pre-switch VBUS_ERRORHI comparator is referenced to the
 * *unloaded* battery voltage: once the pump stops, VBAT falls by IBAT*Rcell
 * (hundreds of mV at 7 A) while the PPS source stays at the loaded operating
 * point, so re-enabling at that point latches the comparator.  Restart from
 * ratio*VBAT(unloaded) + this gap instead, exactly like the downstream
 * pd_cp_manager restarts from its entry_vbus.
 */
#define XAGA_CPM_RESTART_GAP_MV		400
#define XAGA_CPM_RESTART_HYS_MV		80
#define XAGA_CPM_RESTART_IBUS_MA	2000
/*
 * While running, keep a margin below the comparator so a PPS source's
 * transition overshoot does not latch it.  Downstream's entry window is
 * 400..600 mV above ratio*VBAT and the operating point sits well inside it.
 */
#define XAGA_CPM_HR_MAX_MV		450
#define XAGA_CPM_GAP_DOWN_MV		200
#define XAGA_CPM_GAP_UP_MV		100
#define XAGA_CPM_GAP_UP_TICKS		15
#define XAGA_CPM_MIN_HEADROOM_MV 200
#define XAGA_CPM_FCC_START_UA	1500000
/* only parallel the second pump when the target really needs it */
#define XAGA_CPM_SLAVE_MIN_UA	3000000
/* CP-output-to-cell path resistance for the CV cap */
#define XAGA_CPM_PATH_R_MOHM	60
/* hand CV/top-off back to the MT6375 before the cell is full */
#define XAGA_CPM_TOPOFF_SOC	95
/* vendor taper: only hand off once VBAT is above cv_vbat and IBAT below cv_ibat */
#define XAGA_CPM_CV_VBAT_MV	4470
#define XAGA_CPM_CV_IBAT_MA	2000
#define XAGA_CPM_RECHARGE_SOC	75
/* fast-charge power-collapse cutoff */
#define XAGA_CPM_LOWP_MW	8000
#define XAGA_CPM_LOWP_SOC	70
#define XAGA_CPM_LOWP_MS	12000
#define XAGA_CPM_TOPOFF_DWELL_MS 120000

struct xaga_cpm {
	struct device *dev;
	struct power_supply *tcpm;
	struct power_supply *mt6375_psy;
	struct charger_device *cp_master;
	struct charger_device *cp_slave;
	struct charger_device *mt6375;

	u32 max_vbus_uv;
	u32 max_ibus_ua;
	u32 max_fcc_ua;
	u32 min_pdo_uv;
	u32 max_pdo_uv;

	/* closed-loop targets */
	u32 fv_uv;		/* CV target */
	u32 fv_ffc_uv;		/* fast-charge CV target */
	u32 charge_fcc_ua;	/* target battery current */
	u32 target_fcc_ua;	/* ramped toward charge_fcc_ua */
	u32 cable_r_mohm;
	int max_temp;		/* deci-degC hard backstop */
	u32 topoff_soc;		/* hand off to MT6375 at this SOC */
	u32 thermal_level;	/* 0..15, board thermal policy input */
	bool policy_entry;	/* PPS session just started: seed policy */
	struct xaga_cp_tuning tuning;
	struct xaga_cp_policy policy;
	struct power_supply *gauge;
	bool auto_mode;
	/* set from the LK hwid sku stamped into the DT root by setup.c */
	bool xagapro;
	/* charge-pump divider: 2 (SC8551/SC8561 2:1) or 4 (SC8561 4:1) */
	int res;
	int cp_mode;	/* charger_dev_cp_set_mode() value */
	u32 vbus_gap_mv;	/* operating headroom above ratio*VBAT */
	u32 cp_ibus_ua;		/* last measured pump input current */
	u32 cp_vbatt_uv;	/* last measured pump battery voltage */
	int stall_ticks;	/* enabled-but-idle ticks for latch recovery */
	bool stall_hold;	/* keep the pumps off one tick to clear a latch */
	int healthy_ticks;	/* clean ticks before re-widening the window */

	/* requested PPS operating point */
	u32 req_volt_mv;
	u32 req_curr_ma;

	bool pps_on;
	bool pps_tried;
	/* backoff before re-attempting PPS after a source fallback */
	unsigned long pps_retry_jiffies;
	bool cp_on;
	bool cp_auto;
	bool master_on;
	bool slave_on;
	bool mt6375_stopped;
	bool topoff;
	unsigned long topoff_jiffies;
	bool lowp;
	unsigned long lowp_jiffies;
	s64 power_mw_smooth;
	bool force;
	u32 fault_count;
	unsigned long last_req;

	struct mutex lock;
	struct dentry *dbgfs;

	/* periodically re-asserts the PPS operating point */
	struct delayed_work keepalive_work;
	/* closed-loop regulation tick */
	struct delayed_work reg_work;
};

/* ------------------------------------------------------------------ */
/* TCPM helpers                                                       */
/* ------------------------------------------------------------------ */

static int xaga_cpm_tcpm_get(struct xaga_cpm *cpm, enum power_supply_property psp,
			     int *val)
{
	union power_supply_propval p = {};
	int ret;

	ret = power_supply_get_property(cpm->tcpm, psp, &p);
	if (!ret)
		*val = p.intval;
	return ret;
}

static int xaga_cpm_tcpm_set(struct xaga_cpm *cpm, enum power_supply_property psp,
			     int val)
{
	union power_supply_propval p = { .intval = val };

	return power_supply_set_property(cpm->tcpm, psp, &p);
}

static int xaga_cpm_tcpm_online(struct xaga_cpm *cpm, int *val)
{
	return xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_ONLINE, val);
}

static bool xaga_cpm_has_pps(int usb_type)
{
	return usb_type == POWER_SUPPLY_USB_TYPE_PD_PPS ||
	       usb_type == POWER_SUPPLY_USB_TYPE_PD_PPS_SPR_AVS;
}

static int xaga_cpm_gauge_get(struct xaga_cpm *cpm,
			      enum power_supply_property psp, int *val)
{
	union power_supply_propval p = {};
	int ret;

	if (!cpm->gauge)
		return -ENODEV;
	ret = power_supply_get_property(cpm->gauge, psp, &p);
	if (!ret)
		*val = p.intval;
	return ret;
}

/* ------------------------------------------------------------------ */
/* charge-pump / MT6375 control                                       */
/* ------------------------------------------------------------------ */

static int xaga_cpm_cp_apply_one(struct xaga_cpm *cpm,
				 struct charger_device *cp, bool on)
{
	int ret;

	if (on) {
		ret = charger_dev_cp_set_mode(cp, cpm->cp_mode);
		if (ret)
			return ret;
	}

	return charger_dev_enable(cp, on);
}

static int xaga_cpm_cp_apply(struct xaga_cpm *cpm, bool on)
{
	int ret, ret2;

	ret = xaga_cpm_cp_apply_one(cpm, cpm->cp_master, on);
	ret2 = xaga_cpm_cp_apply_one(cpm, cpm->cp_slave, on);
	if (!ret)
		ret = ret2;

	cpm->cp_on = on && !ret;
	return ret;
}

/*
 * A pump may only be switched on once the TCPM reports an active PPS
 * contract, or when the tester explicitly sets "force" for a controlled
 * fixed-PDO experiment.
 */
static bool xaga_cpm_cp_allowed(struct xaga_cpm *cpm)
{
	int online = TCPM_PSY_OFFLINE;

	xaga_cpm_tcpm_online(cpm, &online);
	return cpm->force || online == TCPM_PSY_PPS_ONLINE;
}

static int xaga_cpm_mt6375_sync(struct xaga_cpm *cpm, bool on)
{
	union power_supply_propval p = {};
	int ret;

	if (on) {
		p.intval = XAGA_MT6375_AICR;
		power_supply_set_property(cpm->mt6375_psy,
					  POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
					  &p);
		p.intval = 1;
		ret = power_supply_set_property(cpm->mt6375_psy,
						POWER_SUPPLY_PROP_ONLINE, &p);
	} else {
		p.intval = 0;
		ret = power_supply_set_property(cpm->mt6375_psy,
						POWER_SUPPLY_PROP_ONLINE, &p);
	}

	return ret;
}

/* ------------------------------------------------------------------ */
/* PPS keep-alive                                                     */
/* ------------------------------------------------------------------ */

/*
 * Several PD sources (notably the 100 W-class laptop adapters) hard-reset
 * a PPS contract if the sink does not issue a new PPS Request within a
 * short window.  Re-assert the requested operating point periodically.
 * This heartbeat also becomes the regulation tick once the loop closes.
 */
#define XAGA_CPM_KEEPALIVE_MS	2000

static void xaga_cpm_keepalive_work(struct work_struct *work)
{
	struct xaga_cpm *cpm = container_of(to_delayed_work(work),
					    struct xaga_cpm, keepalive_work);
	int online = TCPM_PSY_OFFLINE;
	u32 volt;

	mutex_lock(&cpm->lock);
	xaga_cpm_tcpm_online(cpm, &online);
	volt = cpm->req_volt_mv;
	mutex_unlock(&cpm->lock);

	if (online != TCPM_PSY_PPS_ONLINE)
		return;

	if (volt)
		xaga_cpm_tcpm_set(cpm, POWER_SUPPLY_PROP_VOLTAGE_NOW,
				  volt * 1000);

	schedule_delayed_work(&cpm->keepalive_work,
			      msecs_to_jiffies(XAGA_CPM_KEEPALIVE_MS));
}

static void xaga_cpm_keepalive_start(struct xaga_cpm *cpm)
{
	schedule_delayed_work(&cpm->keepalive_work,
			      msecs_to_jiffies(XAGA_CPM_KEEPALIVE_MS));
}

static void xaga_cpm_keepalive_stop(struct xaga_cpm *cpm)
{
	cancel_delayed_work_sync(&cpm->keepalive_work);
}

/* ------------------------------------------------------------------ */
/* closed-loop regulation                                             */
/* ------------------------------------------------------------------ */

static void xaga_cpm_reg_work(struct work_struct *work)
{
	struct xaga_cpm *cpm = container_of(to_delayed_work(work),
					    struct xaga_cpm, reg_work);
	int online = TCPM_PSY_OFFLINE;
	int usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	int vbat_uv = 0, ibat_ua = 0, temp = 0, soc = 0;
	int cmax_ua = 0, vmax_uv = 0;
	u32 vbus_uv = 0, ibus_m_ua = 0, ibus_s_ua = 0;
	u32 ibus_total_ma, ibus_limit_ma, target_fcc_ma;
	int vbat_mv, fv_mv, ibat_ma;
	int step, s;
	u32 new_volt_mv, new_curr_ma;

	if (!READ_ONCE(cpm->auto_mode))
		return;

	mutex_lock(&cpm->lock);

	if (xaga_cpm_tcpm_online(cpm, &online))
		goto resched;
	xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_USB_TYPE, &usb_type);

	/* ---- automatic attach / detach state machine ---- */
	if (online == TCPM_PSY_OFFLINE) {
		if (cpm->cp_on) {
			xaga_cpm_cp_apply(cpm, false);
			cpm->master_on = false;
			cpm->slave_on = false;
		}
		if (cpm->mt6375_stopped) {
			xaga_cpm_mt6375_sync(cpm, true);
			cpm->mt6375_stopped = false;
		}
		cpm->pps_on = false;
		cpm->pps_tried = false;
		cpm->pps_retry_jiffies = 0;
		cpm->topoff = false;
		cpm->lowp = false;
		cpm->target_fcc_ua = XAGA_CPM_FCC_START_UA;
		xaga_cp_tuning_reset(&cpm->tuning);
		cpm->tuning.entry_soc = -1;
		cpm->policy_entry = false;
		goto resched;
	}

	/*
	 * Not in a PPS session (fixed contract or SPR-AVS).  If the source
	 * supports PPS, first try to switch to a PPS contract; then, whatever
	 * happens, make sure the MT6375 direct path is back on.  Once a PPS
	 * session has stopped the buck, a fallback to a fixed contract or a
	 * failed PPS negotiation used to leave CHG_EN=0, so a plugged phone
	 * net-discharged instead of falling back to normal charging.
	 */
	if (online == TCPM_PSY_FIXED_ONLINE && !cpm->topoff &&
	    !cpm->pps_tried && xaga_cpm_has_pps(usb_type) &&
	    time_after(jiffies, cpm->pps_retry_jiffies)) {
		if (!xaga_cpm_tcpm_set(cpm, POWER_SUPPLY_PROP_ONLINE,
				       TCPM_PSY_PPS_ONLINE)) {
			cpm->pps_on = true;
			cpm->pps_tried = true;
		} else {
			/* source is not ready yet: try again later */
			cpm->pps_retry_jiffies = jiffies +
				msecs_to_jiffies(XAGA_CPM_PPS_RETRY_MS);
		}
	}

	if (online != TCPM_PSY_PPS_ONLINE) {
		/*
		 * If a PPS session just ended, arm exactly one retry after the
		 * source has settled.  Without this, a source that accepts PPS
		 * and then falls back again leaves pps_tried set forever and the
		 * phone stays on the fixed 9 V/3 A path.
		 */
		if (cpm->pps_on || cpm->pps_tried) {
			cpm->pps_on = false;
			cpm->pps_tried = false;
			cpm->pps_retry_jiffies = jiffies +
				msecs_to_jiffies(XAGA_CPM_PPS_RETRY_MS);
		}
		/* PPS is not running: pumps off, MT6375 direct path on */
		if (cpm->cp_on) {
			xaga_cpm_cp_apply(cpm, false);
			cpm->master_on = false;
			cpm->slave_on = false;
		}
		if (cpm->mt6375_stopped) {
			xaga_cpm_mt6375_sync(cpm, true);
			cpm->mt6375_stopped = false;
		}
		goto resched;
	}

	/* PPS is live: hand the load from the buck to the pumps */
	if (!cpm->pps_on) {
		xaga_cpm_keepalive_start(cpm);
		cpm->policy_entry = true;
	}
	cpm->pps_on = true;
	cpm->pps_tried = false;
	cpm->pps_retry_jiffies = 0;

	/*
	 * Pick the divider from the source's real PPS ceiling.  Only while
	 * the pumps are off: the mode register is written by
	 * charger_dev_cp_set_mode() right before each enable, and
	 * re-deciding it under a running converter interrupts soft-start
	 * (the pump then latches off with SS-timeout / VBUS-error).  xaga is
	 * always 2:1; xagapro uses 4:1 when the source can reach ~18 V.
	 */
	if (!cpm->cp_on) {
		xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_VOLTAGE_MAX, &vmax_uv);
		if (!cpm->xagapro) {
			cpm->res = 2;
			cpm->cp_mode = XAGA_CP_MODE_2_1;
		} else if (vmax_uv == 0 ||
			   vmax_uv >= XAGA_CPM_4_1_MIN_VBUS_UV) {
			cpm->res = 4;
			cpm->cp_mode = XAGA_CP_MODE_4_1;
		} else {
			cpm->res = 2;
			cpm->cp_mode = XAGA_CP_MODE_2_1;
		}
	}

	if (!cpm->mt6375_stopped) {
		xaga_cpm_mt6375_sync(cpm, false);
		cpm->mt6375_stopped = true;
	}
	if (cpm->cp_auto) {
		bool want_slave = cpm->target_fcc_ua >= XAGA_CPM_SLAVE_MIN_UA;
		bool recover = cpm->power_mw_smooth > XAGA_CPM_LOWP_MW;
		bool m = false, s = false, skip_enable = false;

		charger_dev_is_enabled(cpm->cp_master, &m);
		charger_dev_is_enabled(cpm->cp_slave, &s);

		/* a latch needs a full tick with the converter off to clear */
		if (cpm->stall_hold) {
			cpm->stall_hold = false;
			skip_enable = true;
		}

		/*
		 * A pump that stops (the part clears its enable bit on a
		 * VBUS_ERROR latch) must not be re-enabled at the loaded
		 * operating point.  VBAT has just fallen by IBAT*Rcell while
		 * the PPS source is still at the loaded voltage, so the
		 * pre-switch comparator sees a huge headroom and latches again.
		 * Drop to ratio*VBAT(unloaded) + gap first, let the source
		 * actually move, and only then restart - this is what the
		 * downstream pd_cp_manager does by tearing the contract down to
		 * 5 V and re-entering from its entry_vbus.
		 */
		if (!m && !s && !cpm->topoff) {
			u32 vbatt_uv = 0, safe_mv, restart_ma;

			if (!charger_dev_cp_get_vbatt(cpm->cp_master,
						      &vbatt_uv) && vbatt_uv) {
				cpm->cp_vbatt_uv = vbatt_uv;
				safe_mv = cpm->res * (vbatt_uv / 1000) +
					  XAGA_CPM_RESTART_GAP_MV;
				safe_mv = clamp_t(u32, safe_mv,
						  cpm->min_pdo_uv / 1000,
						  cpm->max_pdo_uv / 1000);
				if (cpm->req_volt_mv >
				    safe_mv + XAGA_CPM_RESTART_HYS_MV) {
					restart_ma = cpm->req_curr_ma ?
						min(cpm->req_curr_ma,
						    (u32)XAGA_CPM_RESTART_IBUS_MA) :
						1000;
					dev_info_ratelimited(cpm->dev,
						"pumps stopped at %umV, restart at %umV (VBAT %umV)\n",
						cpm->req_volt_mv, safe_mv,
						vbatt_uv / 1000);
					cpm->req_volt_mv = safe_mv;
					cpm->req_curr_ma = restart_ma;
					xaga_cpm_tcpm_set(cpm,
						POWER_SUPPLY_PROP_CURRENT_NOW,
						restart_ma * 1000);
					xaga_cpm_tcpm_set(cpm,
						POWER_SUPPLY_PROP_VOLTAGE_NOW,
						safe_mv * 1000);
					cpm->stall_hold = true;
					cpm->stall_ticks = 0;
					cpm->healthy_ticks = 0;
					skip_enable = true;
				}
			}
		}

		/*
		 * Latch recovery: a pump that trips VBUS_ERRORHI keeps its
		 * enable bit set, so the "!m" retry below never fires and the
		 * phone just discharges.  If a pump is on but drawing almost
		 * nothing while we are still asking for real current, drop the
		 * request and cycle it so it restarts closer to ratio*VBAT.
		 */
		if ((m || s) && want_slave &&
		    cpm->cp_ibus_ua < XAGA_CPM_STALL_IBUS_UA) {
			if (++cpm->stall_ticks >= XAGA_CPM_STALL_TICKS) {
				u32 floor = cpm->min_pdo_uv / 1000;

				if (cpm->req_volt_mv >
				    XAGA_CPM_STALL_BACKOFF_MV)
					cpm->req_volt_mv -=
						XAGA_CPM_STALL_BACKOFF_MV;
				if (cpm->req_volt_mv < floor)
					cpm->req_volt_mv = floor;
				xaga_cpm_cp_apply(cpm, false);
				m = false;
				s = false;
				cpm->master_on = false;
				cpm->slave_on = false;
				cpm->stall_ticks = 0;
				cpm->stall_hold = true;
				cpm->healthy_ticks = 0;
				skip_enable = true;

				if (cpm->vbus_gap_mv >
				    XAGA_CPM_VBUS_GAP_MIN_MV +
				    XAGA_CPM_GAP_DOWN_MV)
					cpm->vbus_gap_mv -=
						XAGA_CPM_GAP_DOWN_MV;
				else
					cpm->vbus_gap_mv =
						XAGA_CPM_VBUS_GAP_MIN_MV;
			}
		} else {
			cpm->stall_ticks = 0;
			if ((m || s) &&
			    cpm->cp_ibus_ua >= XAGA_CPM_STALL_IBUS_UA) {
				if (++cpm->healthy_ticks >=
				    XAGA_CPM_GAP_UP_TICKS) {
					cpm->healthy_ticks = 0;
					if (cpm->vbus_gap_mv <
					    XAGA_CPM_VBUS_GAP_MV)
						cpm->vbus_gap_mv +=
							XAGA_CPM_GAP_UP_MV;
				}
			} else {
				cpm->healthy_ticks = 0;
			}
		}

		/*
		 * Run the slave only while the target needs it; two paralleled
		 * pumps fight at light load and alternately latch off.  Once
		 * the power has collapsed, stop fighting dropouts and let the
		 * top-off detector hand over to the MT6375.
		 */
		if (!skip_enable && !m && (recover || !cpm->master_on))
			xaga_cpm_cp_apply_one(cpm, cpm->cp_master, true);
		if (!skip_enable && want_slave && !s && recover)
			xaga_cpm_cp_apply_one(cpm, cpm->cp_slave, true);
		if (!want_slave && s)
			xaga_cpm_cp_apply_one(cpm, cpm->cp_slave, false);

		/* keep the reported state equal to the hardware state */
		charger_dev_is_enabled(cpm->cp_master, &m);
		charger_dev_is_enabled(cpm->cp_slave, &s);
		cpm->master_on = m;
		cpm->slave_on = s;
		cpm->cp_on = m || s;
	}

	/* ---- closed-loop regulation ---- */
	if (xaga_cpm_gauge_get(cpm, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv) ||
	    xaga_cpm_gauge_get(cpm, POWER_SUPPLY_PROP_CURRENT_NOW, &ibat_ua) ||
	    xaga_cpm_gauge_get(cpm, POWER_SUPPLY_PROP_TEMP, &temp) ||
	    xaga_cpm_gauge_get(cpm, POWER_SUPPLY_PROP_CAPACITY, &soc))
		goto resched;

	charger_dev_get_vbus(cpm->cp_master, &vbus_uv);
	charger_dev_get_ibus(cpm->cp_master, &ibus_m_ua);
	charger_dev_get_ibus(cpm->cp_slave, &ibus_s_ua);
	cpm->cp_ibus_ua = ibus_m_ua + ibus_s_ua;
	{
		u32 cp_vbatt_uv = 0;

		if (!charger_dev_cp_get_vbatt(cpm->cp_master, &cp_vbatt_uv) &&
		    cp_vbatt_uv)
			cpm->cp_vbatt_uv = cp_vbatt_uv;
	}

	/*
	 * The source's real PPS envelope.  Requests above max_curr are
	 * rejected with -EINVAL and silently leave the old contract.
	 */
	xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_CURRENT_MAX, &cmax_ua);
	xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_VOLTAGE_MAX, &vmax_uv);

	vbat_mv = vbat_uv / 1000;
	ibat_ma = ibat_ua / 1000;
	if (ibat_ma < 0)
		ibat_ma = 0;

	/*
	 * Vendor charging policy: JEITA FCC/FV, VBAT step-charge (low/high
	 * cycle), FFC gating, thermal table and the CV taper.  It replaces
	 * the old linear SOC ramp and the fv+20 mV hand-off.
	 */
	{
		struct xaga_cp_inputs in;
		int cycle_count = 0;

		xaga_cpm_gauge_get(cpm, POWER_SUPPLY_PROP_CYCLE_COUNT,
				   &cycle_count);
		if (cpm->policy_entry) {
			xaga_cp_tuning_entry(&cpm->tuning, soc, vbat_mv, temp);
			cpm->policy_entry = false;
		}
		memset(&in, 0, sizeof(in));
		in.vbat_mv = vbat_mv;
		in.ichg_ma = ibat_ma;
		in.tbat_dc = temp;
		in.soc = soc;
		in.cycle_count = cycle_count;
		in.thermal_level = cpm->thermal_level;
		in.pd_verified = false;		/* no batterysecret yet */
		in.gauge_authentic = true;
		in.charge_full = false;
		xaga_cp_policy_update(&cpm->tuning, &in, &cpm->policy);
	}

	fv_mv = cpm->policy.fv_mv;
	if (fv_mv <= 0)
		fv_mv = cpm->fv_ffc_uv / 1000;
	if (cpm->fv_ffc_uv && fv_mv > cpm->fv_ffc_uv / 1000)
		fv_mv = cpm->fv_ffc_uv / 1000;

	/*
	 * Top-off handoff: before the cell reaches FV, stop the pumps and
	 * give CV/EOC back to the MT6375 (proper CV loop + termination).
	 * This is also the hard over-voltage backstop.
	 */
	/*
	 * If the pumps are up but delivering < 8 W while the pack is above
	 * 70%, they are no longer buying anything; hand CV/EOC to the
	 * MT6375 after it has persisted for a minute.
	 */
	{
		/* uV * uA = pW; /1e9 -> mW */
		s64 power_mw = (s64)vbat_uv * ibat_ua / 1000000000;

		/* smooth over ~8 ticks so the pump's on/off bounce cannot
		 * restart the cutoff timer */
		cpm->power_mw_smooth = cpm->power_mw_smooth ?
			(cpm->power_mw_smooth * 7 + power_mw) / 8 : power_mw;

		if (soc > XAGA_CPM_LOWP_SOC &&
		    cpm->power_mw_smooth < XAGA_CPM_LOWP_MW) {
			if (!cpm->lowp)
				cpm->lowp_jiffies = jiffies;
			cpm->lowp = true;
		} else if (soc <= XAGA_CPM_LOWP_SOC ||
			   cpm->power_mw_smooth > XAGA_CPM_LOWP_MW + 1000) {
			cpm->lowp = false;
		}
	}

	if (!cpm->topoff &&
	    (soc >= (int)cpm->topoff_soc ||
	     cpm->policy.taper ||
	     (cpm->lowp && time_after(jiffies, cpm->lowp_jiffies +
				      msecs_to_jiffies(XAGA_CPM_LOWP_MS))))) {
		cpm->topoff = true;
		cpm->topoff_jiffies = jiffies;
		cpm->pps_tried = true;
		dev_info(cpm->dev,
			 "SOC %d%% (VBAT %dmV IBAT %dmA): handing CV/top-off back to MT6375\n",
			 soc, vbat_mv, ibat_ma);
	}
	if (cpm->topoff && soc <= XAGA_CPM_RECHARGE_SOC &&
	    time_after(jiffies, cpm->topoff_jiffies +
		       msecs_to_jiffies(XAGA_CPM_TOPOFF_DWELL_MS))) {
		cpm->topoff = false;
		cpm->pps_tried = false;
		cpm->target_fcc_ua = XAGA_CPM_FCC_START_UA;
	}

	if (cpm->topoff) {
		if (cpm->cp_on) {
			xaga_cpm_cp_apply(cpm, false);
			cpm->master_on = false;
			cpm->slave_on = false;
		}
		if (cpm->mt6375_stopped) {
			xaga_cpm_mt6375_sync(cpm, true);
			cpm->mt6375_stopped = false;
		}
		if (cpm->pps_on) {
			xaga_cpm_tcpm_set(cpm, POWER_SUPPLY_PROP_ONLINE,
					  TCPM_PSY_FIXED_ONLINE);
			cpm->pps_on = false;
		}
		goto resched;
	}

	/* vendor step/JEITA/thermal/FFC policy -> FCC target */
	cpm->target_fcc_ua = cpm->policy.target_fcc_ma * 1000;
	/* debugfs 'target_fcc' remains a manual cap for bring-up tests */
	if (cpm->charge_fcc_ua &&
	    cpm->target_fcc_ua > cpm->charge_fcc_ua)
		cpm->target_fcc_ua = cpm->charge_fcc_ua;
	/* hard 48 degC backstop, independent of the table */
	if (temp > cpm->max_temp && cpm->target_fcc_ua > 2000000)
		cpm->target_fcc_ua = 2000000;
	if (cpm->target_fcc_ua > cpm->max_fcc_ua)
		cpm->target_fcc_ua = cpm->max_fcc_ua;
	target_fcc_ma = cpm->target_fcc_ua / 1000;

	ibus_total_ma = (ibus_m_ua + ibus_s_ua) / 1000;
	ibus_limit_ma = min(target_fcc_ma / cpm->res +
			    XAGA_CPM_IBUS_GAP_MA,
			    cpm->max_ibus_ua / 1000);
	if (cmax_ua > 0)
		ibus_limit_ma = min(ibus_limit_ma, (u32)cmax_ua / 1000);

	/* vbat loop: drive VBAT to FV */
	if (fv_mv - vbat_mv > 400)
		step = 5;
	else if (fv_mv - vbat_mv > 200)
		step = 2;
	else if (fv_mv - vbat_mv > 5)
		step = 1;
	else if (fv_mv - vbat_mv < -2)
		step = -2;
	else if (fv_mv - vbat_mv < 0)
		step = -1;
	else
		step = 0;

	/* ibat loop: drive IBAT to the (ramped) target */
	if ((int)target_fcc_ma - ibat_ma > 400)
		s = 5;
	else if ((int)target_fcc_ma - ibat_ma > 200)
		s = 2;
	else if ((int)target_fcc_ma - ibat_ma > 50)
		s = 1;
	else if (ibat_ma > (int)target_fcc_ma + 100)
		s = -5;
	else if (ibat_ma > (int)target_fcc_ma + 50)
		s = -2;
	else
		s = 0;
	if (s < step)
		step = s;

	/* ibus loop: keep the input current under the contract target */
	if ((int)ibus_limit_ma - (int)ibus_total_ma > 400)
		s = 5;
	else if ((int)ibus_limit_ma - (int)ibus_total_ma > 200)
		s = 2;
	else if ((int)ibus_limit_ma - (int)ibus_total_ma > 0)
		s = 1;
	else if ((int)ibus_total_ma > (int)ibus_limit_ma + 100)
		s = -5;
	else
		s = 0;
	if (s < step)
		step = s;

	/*
	 * Keep a margin below the pump's VBUS_ERRORHI comparator while it is
	 * running: a PPS source overshoots on a voltage step, and the part
	 * latches if the input sits too far above ratio*VBAT.  With the pumps
	 * off the measured VBUS is the unloaded source voltage, so this also
	 * stops the loop from raising the request before the restart window
	 * has been re-established.
	 */
	if (cpm->cp_vbatt_uv) {
		int hr_mv = (int)(vbus_uv / 1000) -
			    cpm->res * (int)(cpm->cp_vbatt_uv / 1000);

		if (hr_mv > XAGA_CPM_HR_MAX_MV)
			step = min(step, 0);
	}

	/* never go above the configured maximum VBUS */
	if (cpm->req_volt_mv >= cpm->max_vbus_uv / 1000)
		step = min(step, 0);

	new_volt_mv = cpm->req_volt_mv + step * XAGA_CPM_STEP_MV;

	{
		/*
		 * Output path drop: Vcp_out = VBAT + I*Rpath.
		 */
		u32 drop_mv = (u32)ibat_ma * XAGA_CPM_PATH_R_MOHM / 1000;
		/*
		 * Only keep the pump above its minimum headroom while we
		 * are raising the voltage.  When the loops want to reduce
		 * (CV), they must be allowed to bring Vcp_out down to FV,
		 * otherwise the clamp keeps pushing current into a full
		 * cell.
		 */
		u32 min_oper_mv = cpm->res *
				  (u32)(vbat_mv + XAGA_CPM_MIN_HEADROOM_MV);
		/*
		 * CV cap: Vcp_out <= FV + I*Rpath, i.e. the cell terminal
		 * reaches FV at the present current and tapers to FV as I
		 * falls.
		 */
		u32 cap_mv = cpm->res * (u32)fv_mv + 2 * drop_mv;

		/*
		 * Both pumps latch off with VBUS_ERRORHI when the input sits
		 * too far above ratio*VBAT.  That happens at a very low cell
		 * (the fixed request becomes too much headroom) or when the
		 * loop runs the voltage away while no current flows.  Cap the
		 * request at ratio*VBAT + gap instead of the ratio*FV CV
		 * ceiling.  The window rises with the cell, so it still
		 * converges to CV at the top.
		 */
		{
			u32 op_cap = cpm->res * (u32)vbat_mv +
				     cpm->vbus_gap_mv;

			if (cap_mv > op_cap)
				cap_mv = op_cap;
		}

		if (step >= 0 && new_volt_mv < min_oper_mv)
			new_volt_mv = min_oper_mv;
		if (new_volt_mv > cap_mv)
			new_volt_mv = cap_mv;
		if (vmax_uv > 0 && new_volt_mv > (u32)vmax_uv / 1000)
			new_volt_mv = (u32)vmax_uv / 1000;
		new_volt_mv = clamp_t(u32, new_volt_mv,
				      cpm->min_pdo_uv / 1000,
				      cpm->max_pdo_uv / 1000);
	}

	new_curr_ma = ibus_limit_ma;

	/*
	 * Update the current first: tcpm's voltage request is built from the
	 * already-negotiated current, so this makes the (last) voltage
	 * transition carry both values instead of stepping the source twice
	 * with the old current still in force.
	 */
	if (new_curr_ma != cpm->req_curr_ma) {
		if (!xaga_cpm_tcpm_set(cpm, POWER_SUPPLY_PROP_CURRENT_NOW,
				       new_curr_ma * 1000))
			cpm->req_curr_ma = new_curr_ma;
	}
	if (new_volt_mv != cpm->req_volt_mv ||
	    time_after(jiffies, cpm->last_req +
		       msecs_to_jiffies(XAGA_CPM_KEEPALIVE_MS))) {
		if (!xaga_cpm_tcpm_set(cpm, POWER_SUPPLY_PROP_VOLTAGE_NOW,
				       new_volt_mv * 1000)) {
			cpm->req_volt_mv = new_volt_mv;
			cpm->last_req = jiffies;
		}
	}

resched:
	mutex_unlock(&cpm->lock);
	if (READ_ONCE(cpm->auto_mode))
		schedule_delayed_work(&cpm->reg_work,
				      msecs_to_jiffies(XAGA_CPM_REG_MS));
}

/* ------------------------------------------------------------------ */
/* debugfs                                                            */
/* ------------------------------------------------------------------ */

static int xaga_cpm_caps_show(struct seq_file *s, void *unused)
{
	struct xaga_cpm *cpm = s->private;
	int usb_type = 0, online = 0, vmin = 0, vmax = 0, vnow = 0;
	int cmax = 0, cnow = 0;
	bool cp_m = false, cp_s = false;

	mutex_lock(&cpm->lock);
	xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_USB_TYPE, &usb_type);
	xaga_cpm_tcpm_online(cpm, &online);
	xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_VOLTAGE_MIN, &vmin);
	xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_VOLTAGE_MAX, &vmax);
	xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vnow);
	xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_CURRENT_MAX, &cmax);
	xaga_cpm_tcpm_get(cpm, POWER_SUPPLY_PROP_CURRENT_NOW, &cnow);
	charger_dev_is_enabled(cpm->cp_master, &cp_m);
	charger_dev_is_enabled(cpm->cp_slave, &cp_s);
	mutex_unlock(&cpm->lock);

	seq_printf(s, "board xagapro=%d res=%d cp_mode=%d\n",
		   cpm->xagapro, cpm->res, cpm->cp_mode);
	seq_printf(s, "usb_type=%d online=%d pps_on=%d force=%d\n",
		   usb_type, online, cpm->pps_on, cpm->force);
	seq_printf(s, "cp_on=%d cp_master=%d cp_slave=%d\n",
		   cpm->cp_on, cp_m, cp_s);
	seq_printf(s, "tcpm Vmin=%d Vmax=%d Vnow=%d Cmax=%d Cnow=%d (uV/uA)\n",
		   vmin, vmax, vnow, cmax, cnow);
	seq_printf(s, "limits Vbus_max=%u Ibus_max=%u Fcc_max=%u PDO=%u..%u\n",
		   cpm->max_vbus_uv, cpm->max_ibus_ua, cpm->max_fcc_ua,
		   cpm->min_pdo_uv, cpm->max_pdo_uv);
	seq_printf(s, "requested %u mV / %u mA, faults=%u\n",
		   cpm->req_volt_mv, cpm->req_curr_ma, cpm->fault_count);
	seq_printf(s, "policy step=%d jeita=%d ffc=%d taper=%d lowcycle=%d fv=%dmV target=%dmA\n",
		   cpm->policy.step_index, cpm->policy.jeita_index,
		   cpm->policy.ffc, cpm->policy.taper,
		   cpm->tuning.force_low_cycle,
		   cpm->policy.fv_mv, cpm->policy.target_fcc_ma);
	seq_printf(s, "policy fcc step=%d jeita=%d ffc=%d thermal=%d sw_cv=%d\n",
		   cpm->policy.step_fcc_ma, cpm->policy.jeita_fcc_ma,
		   cpm->policy.ffc_fcc_ma, cpm->policy.thermal_fcc_ma,
		   cpm->policy.sw_cv_mv);
	seq_printf(s, "thermal_level=%u\n", cpm->thermal_level);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(xaga_cpm_caps);

static int xaga_cpm_pps_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;
	int online = TCPM_PSY_OFFLINE;

	xaga_cpm_tcpm_online(cpm, &online);
	*val = online == TCPM_PSY_PPS_ONLINE;
	return 0;
}

static int xaga_cpm_pps_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;
	int mode = val ? TCPM_PSY_PPS_ONLINE : TCPM_PSY_FIXED_ONLINE;
	int ret;

	mutex_lock(&cpm->lock);
	if (val) {
		int vnow = 0;

		/*
		 * tcpm_pps_activate() requests the voltage of the existing
		 * fixed contract.  Seed the keep-alive from it so we re-assert
		 * a usable (MT6375 MIVR-compatible) PPS voltage.
		 */
		if (!xaga_cpm_tcpm_get(cpm,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, &vnow) &&
		    vnow > 0)
			cpm->req_volt_mv = vnow / 1000;
	}
	ret = xaga_cpm_tcpm_set(cpm, POWER_SUPPLY_PROP_ONLINE, mode);
	if (!ret)
		cpm->pps_on = !!val;
	mutex_unlock(&cpm->lock);

	if (!ret) {
		if (val)
			xaga_cpm_keepalive_start(cpm);
		else
			xaga_cpm_keepalive_stop(cpm);
	}

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_pps_fops, xaga_cpm_pps_get,
			 xaga_cpm_pps_set, "%llu\n");

static int xaga_cpm_req_volt_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->req_volt_mv;
	return 0;
}

static int xaga_cpm_req_volt_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;
	int ret;

	if (val < cpm->min_pdo_uv / 1000 || val > cpm->max_pdo_uv / 1000)
		return -EINVAL;

	mutex_lock(&cpm->lock);
	ret = xaga_cpm_tcpm_set(cpm, POWER_SUPPLY_PROP_VOLTAGE_NOW,
				val * 1000);
	if (!ret)
		cpm->req_volt_mv = val;
	mutex_unlock(&cpm->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_req_volt_fops, xaga_cpm_req_volt_get,
			 xaga_cpm_req_volt_set, "%llu\n");

static int xaga_cpm_req_curr_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->req_curr_ma;
	return 0;
}

static int xaga_cpm_req_curr_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;
	int ret;

	if (val > cpm->max_ibus_ua / 1000)
		return -EINVAL;

	mutex_lock(&cpm->lock);
	ret = xaga_cpm_tcpm_set(cpm, POWER_SUPPLY_PROP_CURRENT_NOW,
				val * 1000);
	if (!ret)
		cpm->req_curr_ma = val;
	mutex_unlock(&cpm->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_req_curr_fops, xaga_cpm_req_curr_get,
			 xaga_cpm_req_curr_set, "%llu\n");

static int xaga_cpm_force_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->force;
	return 0;
}

static int xaga_cpm_force_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;

	cpm->force = !!val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_force_fops, xaga_cpm_force_get,
			 xaga_cpm_force_set, "%llu\n");

static int xaga_cpm_auto_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->auto_mode;
	return 0;
}

static int xaga_cpm_auto_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;

	if (val) {
		cpm->target_fcc_ua = XAGA_CPM_FCC_START_UA;
		cpm->auto_mode = true;
		schedule_delayed_work(&cpm->reg_work, 0);
	} else {
		cpm->auto_mode = false;
		cancel_delayed_work_sync(&cpm->reg_work);

		/*
		 * Full teardown so the MT6375 (with its own CV) takes over:
		 * drop the PPS contract, stop the pumps and re-enable the
		 * buck.  Without this the pumps stay latched on the last
		 * PPS point.
		 */
		mutex_lock(&cpm->lock);
		if (cpm->cp_on) {
			xaga_cpm_cp_apply(cpm, false);
			cpm->master_on = false;
			cpm->slave_on = false;
		}
		if (cpm->mt6375_stopped) {
			xaga_cpm_mt6375_sync(cpm, true);
			cpm->mt6375_stopped = false;
		}
		if (cpm->pps_on) {
			xaga_cpm_tcpm_set(cpm, POWER_SUPPLY_PROP_ONLINE,
					  TCPM_PSY_FIXED_ONLINE);
			cpm->pps_on = false;
			cpm->pps_tried = false;
		}
		mutex_unlock(&cpm->lock);
	}

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_auto_fops, xaga_cpm_auto_get,
			 xaga_cpm_auto_set, "%llu\n");

/*
 * Battery charge-voltage ceiling (mV).  Caps the charge-pump CV and programs
 * the MT6375 CV, so a battery-lifetime limit holds on both paths (e.g. a
 * home server parked at 3.9 V).  The CP keeps doing the fast-charge work
 * below the cap.
 */
static int xaga_cpm_cv_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->fv_ffc_uv / 1000;
	return 0;
}

static int xaga_cpm_cv_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;
	struct charger_device *chg;
	u32 uv = (u32)val * 1000;
	int ret = 0;

	if (val < 3000 || val > 4600)
		return -EINVAL;

	chg = cpm->mt6375;
	if (!chg) {
		chg = get_charger_by_name("primary_chg");
		cpm->mt6375 = chg;
	}

	mutex_lock(&cpm->lock);
	cpm->fv_uv = uv;
	cpm->fv_ffc_uv = uv;
	if (chg)
		ret = charger_dev_set_constant_voltage(chg, uv);
	mutex_unlock(&cpm->lock);

	dev_info(cpm->dev, "cv cap %u uV (charger %s, ret %d)\n", uv,
		 chg ? "found" : "missing", ret);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_cv_fops, xaga_cpm_cv_get,
			 xaga_cpm_cv_set, "%llu\n");

static int xaga_cpm_fcc_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->charge_fcc_ua;
	return 0;
}

static int xaga_cpm_fcc_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;

	if (val > cpm->max_fcc_ua)
		return -EINVAL;
	cpm->charge_fcc_ua = val;
	if (cpm->target_fcc_ua > val)
		cpm->target_fcc_ua = val;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_fcc_fops, xaga_cpm_fcc_get,
			 xaga_cpm_fcc_set, "%llu\n");

static int xaga_cpm_vbus_gap_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->vbus_gap_mv;
	return 0;
}

static int xaga_cpm_vbus_gap_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;

	if (val > 4000)
		return -EINVAL;
	cpm->vbus_gap_mv = val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_vbus_gap_fops, xaga_cpm_vbus_gap_get,
			 xaga_cpm_vbus_gap_set, "%llu\n");

static int xaga_cpm_topoff_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->topoff_soc;
	return 0;
}

static int xaga_cpm_topoff_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;

	if (val > 100)
		return -EINVAL;
	cpm->topoff_soc = val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_topoff_fops, xaga_cpm_topoff_get,
			 xaga_cpm_topoff_set, "%llu\n");

static int xaga_cpm_thermal_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->thermal_level;
	return 0;
}

static int xaga_cpm_thermal_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;

	if (val > 15)
		return -EINVAL;
	cpm->thermal_level = val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_thermal_fops, xaga_cpm_thermal_get,
			 xaga_cpm_thermal_set, "%llu\n");

static int xaga_cpm_lowcycle_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->tuning.force_low_cycle;
	return 0;
}

static int xaga_cpm_lowcycle_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;

	mutex_lock(&cpm->lock);
	cpm->tuning.force_low_cycle = !!val;
	/* re-seed the table index so the change takes effect immediately */
	xaga_cp_tuning_reset(&cpm->tuning);
	cpm->tuning.entry_soc = -1;
	mutex_unlock(&cpm->lock);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_lowcycle_fops, xaga_cpm_lowcycle_get,
			 xaga_cpm_lowcycle_set, "%llu\n");

static int xaga_cpm_mt6375_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;
	union power_supply_propval p = {};
	int ret;

	ret = power_supply_get_property(cpm->mt6375_psy,
					POWER_SUPPLY_PROP_ONLINE, &p);
	if (!ret)
		*val = p.intval;
	return ret;
}

static int xaga_cpm_mt6375_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;
	int ret;

	mutex_lock(&cpm->lock);
	ret = xaga_cpm_mt6375_sync(cpm, !!val);
	mutex_unlock(&cpm->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_mt6375_fops, xaga_cpm_mt6375_get,
			 xaga_cpm_mt6375_set, "%llu\n");

static int xaga_cpm_cp_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = cpm->cp_on;
	return 0;
}

static int xaga_cpm_cp_set(void *data, u64 val)
{
	struct xaga_cpm *cpm = data;
	int ret;

	mutex_lock(&cpm->lock);
	if (val && !xaga_cpm_cp_allowed(cpm)) {
		ret = -EAGAIN;
	} else {
		ret = xaga_cpm_cp_apply(cpm, !!val);
		if (!ret) {
			cpm->master_on = !!val;
			cpm->slave_on = !!val;
		}
	}
	mutex_unlock(&cpm->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_cp_fops, xaga_cpm_cp_get,
			 xaga_cpm_cp_set, "%llu\n");

#define XAGA_CPM_CP_FOPS(_name, _cp, _flag)			\
	static int xaga_cpm_##_name##_get(void *data, u64 *val)		\
	{								\
		struct xaga_cpm *cpm = data;				\
		bool en;						\
		int ret;						\
									\
		ret = charger_dev_is_enabled(cpm->_cp, &en);		\
		if (!ret)						\
			*val = en;					\
		return ret;						\
	}								\
	static int xaga_cpm_##_name##_set(void *data, u64 val)		\
	{								\
		struct xaga_cpm *cpm = data;				\
		int ret;						\
									\
		mutex_lock(&cpm->lock);					\
		if (val && !xaga_cpm_cp_allowed(cpm)) {			\
			ret = -EAGAIN;					\
		} else {						\
			ret = xaga_cpm_cp_apply_one(cpm, cpm->_cp, !!val); \
			if (!ret)					\
				cpm->_flag = !!val;			\
		}							\
		mutex_unlock(&cpm->lock);				\
		return ret;						\
	}								\
	DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_##_name##_fops,		\
				 xaga_cpm_##_name##_get,		\
				 xaga_cpm_##_name##_set, "%llu\n")

XAGA_CPM_CP_FOPS(cp_master, cp_master, master_on);
XAGA_CPM_CP_FOPS(cp_slave, cp_slave, slave_on);

static int xaga_cpm_faults_get(void *data, u64 *val)
{
	struct xaga_cpm *cpm = data;

	*val = READ_ONCE(cpm->fault_count);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(xaga_cpm_faults_fops, xaga_cpm_faults_get,
			 NULL, "%llu\n");

static void xaga_cpm_debugfs_init(struct xaga_cpm *cpm)
{
	cpm->dbgfs = debugfs_create_dir("xaga_cp_manager", NULL);
	debugfs_create_file("caps", 0400, cpm->dbgfs, cpm,
			    &xaga_cpm_caps_fops);
	debugfs_create_file("pps", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_pps_fops);
	debugfs_create_file("req_volt", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_req_volt_fops);
	debugfs_create_file("req_curr", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_req_curr_fops);
	debugfs_create_file("force", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_force_fops);
	debugfs_create_file("auto", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_auto_fops);
	debugfs_create_file("target_fcc", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_fcc_fops);
	debugfs_create_file("topoff_soc", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_topoff_fops);
	debugfs_create_file("thermal_level", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_thermal_fops);
	debugfs_create_file("force_low_cycle", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_lowcycle_fops);
	debugfs_create_file("vbus_gap", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_vbus_gap_fops);
	debugfs_create_file("cv", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_cv_fops);
	debugfs_create_file("mt6375", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_mt6375_fops);
	debugfs_create_file("cp", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_cp_fops);
	debugfs_create_file("cp_master", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_cp_master_fops);
	debugfs_create_file("cp_slave", 0600, cpm->dbgfs, cpm,
			    &xaga_cpm_cp_slave_fops);
	debugfs_create_file("faults", 0400, cpm->dbgfs, cpm,
			    &xaga_cpm_faults_fops);
}

/* ------------------------------------------------------------------ */
/* probe / remove                                                     */
/* ------------------------------------------------------------------ */

static int xaga_cpm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xaga_cpm *cpm;

	cpm = devm_kzalloc(dev, sizeof(*cpm), GFP_KERNEL);
	if (!cpm)
		return -ENOMEM;

	cpm->dev = dev;
	mutex_init(&cpm->lock);

	/*
	 * The board DTS is shared across the xaga family; setup.c patches the
	 * root "model" from LK's hwid.sku before dropping the cmdline, so read
	 * the sku it now also publishes to pick the variant.
	 */
	{
		const char *sku = NULL;
		const char *country = NULL;

		of_property_read_string(of_root, "xiaomi,hwid-sku", &sku);
		of_property_read_string(of_root, "xiaomi,hwid-country",
					&country);
		cpm->xagapro = sku && !strcmp(sku, "xagapro");
		/* probe can defer; announce the variant only once */
		dev_info_once(dev, "board sku=%s country=%s%s\n",
			      sku ?: "<unknown>", country ?: "<unknown>",
			      cpm->xagapro ? " (xagapro)" : "");
	}

	INIT_DELAYED_WORK(&cpm->keepalive_work, xaga_cpm_keepalive_work);
	INIT_DELAYED_WORK(&cpm->reg_work, xaga_cpm_reg_work);

	xaga_cp_tuning_init(&cpm->tuning, cpm->xagapro);
	{
		struct device_node *tnp = dev->of_node;
		struct device_node *child = NULL;

		if (cpm->xagapro) {
			child = of_get_child_by_name(dev->of_node, "xagapro");
			if (child)
				tnp = child;
		}
		xaga_cp_tuning_parse(dev, tnp, &cpm->tuning);
		of_node_put(child);
	}
	cpm->thermal_level = 0;
	cpm->policy_entry = false;

	/* defaults mirror the downstream xaga pd_cp_manager node */
	cpm->max_vbus_uv = 12000000;
	cpm->max_ibus_ua = 6200000;
	cpm->max_fcc_ua = 12400000;
	cpm->min_pdo_uv = 8000000;
	cpm->max_pdo_uv = 11000000;

	device_property_read_u32(dev, "max-vbus-microvolt", &cpm->max_vbus_uv);
	device_property_read_u32(dev, "max-ibus-microamp", &cpm->max_ibus_ua);
	device_property_read_u32(dev, "max-fcc-microamp", &cpm->max_fcc_ua);
	device_property_read_u32(dev, "min-pdo-microvolt", &cpm->min_pdo_uv);
	device_property_read_u32(dev, "max-pdo-microvolt", &cpm->max_pdo_uv);

	/* closed-loop defaults (xaga pd_cp_manager values) */
	cpm->fv_uv = 4450000;
	cpm->fv_ffc_uv = 4480000;
	cpm->charge_fcc_ua = min(cpm->max_fcc_ua, 6000000U);
	cpm->cable_r_mohm = 350;
	cpm->max_temp = 450;
	cpm->topoff_soc = XAGA_CPM_TOPOFF_SOC;
	device_property_read_u32(dev, "topoff-soc-percent", &cpm->topoff_soc);
	device_property_read_u32(dev, "constant-charge-voltage-microvolt",
				 &cpm->fv_uv);
	device_property_read_u32(dev, "charge-fcc-microamp",
				 &cpm->charge_fcc_ua);
	device_property_read_u32(dev, "constant-charge-voltage-ffc-microvolt",
				 &cpm->fv_ffc_uv);
	device_property_read_u32(dev, "cable-resistance-milliohm",
				 &cpm->cable_r_mohm);
	{
		u32 max_temp = (u32)cpm->max_temp;

		device_property_read_u32(dev, "max-temp-decidegrees",
					 &max_temp);
		cpm->max_temp = max_temp;
	}

	/*
	 * xagapro differs only in the charge path: SC8561 pumps with a 4:1
	 * option, a higher input envelope and FFC voltage.  Its values live
	 * in the "xagapro" child node so the shared board DTB documents both
	 * variants; the driver picks it from the LK hwid sku.
	 */
	if (cpm->xagapro) {
		struct device_node *np =
			of_get_child_by_name(dev->of_node, "xagapro");

		if (np) {
			of_property_read_u32(np, "max-vbus-microvolt",
					     &cpm->max_vbus_uv);
			of_property_read_u32(np, "max-ibus-microamp",
					     &cpm->max_ibus_ua);
			of_property_read_u32(np, "max-fcc-microamp",
					     &cpm->max_fcc_ua);
			of_property_read_u32(np, "min-pdo-microvolt",
					     &cpm->min_pdo_uv);
			of_property_read_u32(np, "max-pdo-microvolt",
					     &cpm->max_pdo_uv);
			of_property_read_u32(np,
					     "constant-charge-voltage-microvolt",
					     &cpm->fv_uv);
			of_property_read_u32(np,
					     "constant-charge-voltage-ffc-microvolt",
					     &cpm->fv_ffc_uv);
			of_property_read_u32(np, "charge-fcc-microamp",
					     &cpm->charge_fcc_ua);
			of_property_read_u32(np, "cable-resistance-milliohm",
					     &cpm->cable_r_mohm);
			of_property_read_u32(np, "topoff-soc-percent",
					     &cpm->topoff_soc);
			of_node_put(np);
		} else {
			dev_warn(dev, "xagapro: missing DT envelope, using xaga limits\n");
		}
	}

	/* SC8561 can divide by 4; the SC8551A pair is fixed 2:1 */
	cpm->res = cpm->xagapro ? 4 : 2;
	cpm->cp_mode = cpm->xagapro ? XAGA_CP_MODE_4_1 : XAGA_CP_MODE_2_1;
	cpm->vbus_gap_mv = XAGA_CPM_VBUS_GAP_MV;

	cpm->tcpm = devm_power_supply_get_by_reference(dev, "power-supplies");
	if (!cpm->tcpm)
		return -EPROBE_DEFER;
	if (IS_ERR(cpm->tcpm))
		return dev_err_probe(dev, PTR_ERR(cpm->tcpm),
				     "failed to get tcpm supply\n");

	cpm->mt6375_psy = devm_power_supply_get_by_reference(dev,
							     "charger-supply");
	if (!cpm->mt6375_psy)
		return -EPROBE_DEFER;
	if (IS_ERR(cpm->mt6375_psy))
		return dev_err_probe(dev, PTR_ERR(cpm->mt6375_psy),
				     "failed to get charger supply\n");

	cpm->gauge = devm_power_supply_get_by_reference(dev, "gauge-supply");
	if (!cpm->gauge)
		return -EPROBE_DEFER;
	if (IS_ERR(cpm->gauge))
		return dev_err_probe(dev, PTR_ERR(cpm->gauge),
				     "failed to get gauge supply\n");

	cpm->cp_master = get_charger_by_name("cp_master");
	if (!cpm->cp_master)
		return -EPROBE_DEFER;
	cpm->cp_slave = get_charger_by_name("cp_slave");
	if (!cpm->cp_slave)
		return -EPROBE_DEFER;

	/* optional; the MT6375 remains TCPM-managed for the direct path */
	cpm->mt6375 = get_charger_by_name("primary_chg");

	cpm->req_volt_mv = cpm->min_pdo_uv / 1000;
	cpm->req_curr_ma = 0;
	cpm->target_fcc_ua = XAGA_CPM_FCC_START_UA;
	cpm->last_req = jiffies;
	/* automatic fast charge is on by default; debugfs "auto" can stop it */
	cpm->auto_mode = true;
	cpm->cp_auto = true;

	platform_set_drvdata(pdev, cpm);
	mutex_lock(&cpm->lock);
	xaga_cpm_cp_apply(cpm, false);
	mutex_unlock(&cpm->lock);

	xaga_cpm_debugfs_init(cpm);
	schedule_delayed_work(&cpm->reg_work, 0);

	dev_info(dev, "xaga CP manager ready (%s res=%d limit Vbus=%u Ibus=%u Fcc=%u, PDO=%u..%u uV)\n",
		 cpm->xagapro ? "xagapro" : "xaga", cpm->res,
		 cpm->max_vbus_uv, cpm->max_ibus_ua, cpm->max_fcc_ua,
		 cpm->min_pdo_uv, cpm->max_pdo_uv);

	return 0;
}

static void xaga_cpm_remove(struct platform_device *pdev)
{
	struct xaga_cpm *cpm = platform_get_drvdata(pdev);

	cpm->auto_mode = false;
	cancel_delayed_work_sync(&cpm->reg_work);
	xaga_cpm_keepalive_stop(cpm);
	mutex_lock(&cpm->lock);
	xaga_cpm_cp_apply(cpm, false);
	mutex_unlock(&cpm->lock);
	debugfs_remove_recursive(cpm->dbgfs);
}

static const struct of_device_id xaga_cpm_of_match[] = {
	{ .compatible = "xiaomi,xaga-cp-manager" },
	{}
};
MODULE_DEVICE_TABLE(of, xaga_cpm_of_match);

static struct platform_driver xaga_cpm_driver = {
	.driver = {
		.name = "xaga-cp-manager",
		.of_match_table = xaga_cpm_of_match,
	},
	.probe = xaga_cpm_probe,
	.remove = xaga_cpm_remove,
};
module_platform_driver(xaga_cpm_driver);

MODULE_AUTHOR("xaga mainline port");
MODULE_DESCRIPTION("xaga charge-pump manager");
MODULE_LICENSE("GPL");
