/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MTK_DVFSRC_MULTIMEDIA_H
#define __MTK_DVFSRC_MULTIMEDIA_H

/* Caller supplies errno definitions and serializes all transactions. */
#define MTK_MM_STEPS 5
#define MTK_MM_MUXES 7

struct mtk_mm_state {
	unsigned int step;
	int faulted;
	int recovery_error;
};

struct mtk_mm_ops {
	int (*voltage)(void *ctx, unsigned int step);
	int (*parent)(void *ctx, unsigned int mux, unsigned int step);
};

static inline int mtk_mm_transition(struct mtk_mm_state *state,
				    const struct mtk_mm_ops *ops, void *ctx,
				    unsigned int step)
{
	unsigned int i;
	int ret;

	if (step >= MTK_MM_STEPS)
		return -EINVAL;
	if (state->faulted)
		return -EIO;

	/* Also synchronize all parents on the initial, equal-step handover. */
	if (step >= state->step) {
		ret = ops->voltage(ctx, step);
		if (ret)
			goto fault;
	}
	for (i = 0; i < MTK_MM_MUXES; i++) {
		ret = ops->parent(ctx, i, step);
		if (ret)
			goto fault;
	}
	if (step < state->step) {
		ret = ops->voltage(ctx, step);
		if (ret)
			goto fault;
	}
	state->step = step;
	return 0;

fault:
	/*
	 * A failed voltage request may already have reached hardware. Never
	 * restore fast parents based on the old software selector. Request the
	 * highest rail, leave parents alone, and reject further transactions.
	 * If recovery also fails, report that loss of hardware confirmation.
	 */
	state->faulted = 1;
	state->recovery_error = ops->voltage(ctx, MTK_MM_STEPS - 1);
	return ret;
}

#endif
