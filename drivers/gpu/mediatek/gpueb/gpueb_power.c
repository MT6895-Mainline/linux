// SPDX-License-Identifier: GPL-2.0
/*
 * MT6895 GPUEB power handshake for mainline panthor.
 *
 * This is the minimal client needed by the Mali CSF driver: tell the GPUEB
 * firmware where the shared status/debug memory is, then ask it to release
 * the GPU for MCU boot.
 */

#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/soc/mediatek/mt6895_gpueb.h>
#include <linux/soc/mediatek/mtk_tinysys_ipi.h>

#include "gpueb_ipi.h"
#include "gpueb_reserved_mem.h"
#include "gpufreq_ipi.h"

static DEFINE_MUTEX(gpueb_power_lock);
static struct gpufreq_ipi_data gpueb_power_recv_msg;
static int gpueb_power_channel = -1;
static bool gpueb_power_registered;

static int gpueb_power_init_locked(void)
{
	struct gpufreq_ipi_data msg = {};
	phys_addr_t shared_mem_pa;
	phys_addr_t shared_mem_size;
	int ret;

	if (gpueb_power_registered)
		return 0;

	if (!gpueb_ipidev.ipi_inited)
		return -EPROBE_DEFER;

	gpueb_power_channel = gpueb_get_send_PIN_ID_by_name("IPI_ID_GPUFREQ");
	if (gpueb_power_channel < 0)
		return -ENODEV;

	ret = mtk_ipi_register(&gpueb_ipidev, gpueb_power_channel, NULL, NULL,
			       &gpueb_power_recv_msg);
	if (ret != IPI_ACTION_DONE && ret != IPI_DUPLEX)
		return -EIO;

	shared_mem_pa = gpueb_get_reserve_mem_phys_by_name("MEM_ID_GPUFREQ");
	shared_mem_size = gpueb_get_reserve_mem_size_by_name("MEM_ID_GPUFREQ");
	if (!shared_mem_pa || !shared_mem_size)
		return -ENOMEM;

	msg.cmd_id = CMD_INIT_SHARED_MEM;
	msg.u.addr.status_base = shared_mem_pa;
	msg.u.addr.status_size = GPUFREQ_STATUS_MEM_SZ;
	msg.u.addr.debug_base = shared_mem_pa + GPUFREQ_STATUS_MEM_SZ;
	msg.u.addr.debug_size = shared_mem_size - GPUFREQ_STATUS_MEM_SZ;

	ret = mtk_ipi_send_compl(&gpueb_ipidev, gpueb_power_channel,
				 IPI_SEND_POLLING, &msg,
				 GPUFREQ_IPI_DATA_LEN, IPI_TIMEOUT_MS);
	if (ret != IPI_ACTION_DONE)
		return -EIO;

	gpueb_power_registered = true;
	pr_info("XAGA-GPUEB: CMD_INIT_SHARED_MEM done\n");
	return 0;
}



static int gpueb_power_commit_locked(int target, int oppidx)
{
	struct gpufreq_ipi_data msg = {};
	int ret;

	msg.cmd_id = CMD_COMMIT;
	msg.target = target;
	msg.u.oppidx = oppidx;

	ret = mtk_ipi_send_compl(&gpueb_ipidev, gpueb_power_channel,
				 IPI_SEND_POLLING, &msg,
				 GPUFREQ_IPI_DATA_LEN, IPI_TIMEOUT_MS);
	if (ret != IPI_ACTION_DONE)
		return -EIO;

	if (gpueb_power_recv_msg.u.return_value < 0)
		return gpueb_power_recv_msg.u.return_value;

	return 0;
}

bool mt6895_gpueb_available(void)
{
	struct device_node *np;
	bool available;

	np = of_find_compatible_node(NULL, NULL, "mediatek,gpueb");
	available = np != NULL;
	of_node_put(np);

	return available;
}
EXPORT_SYMBOL_GPL(mt6895_gpueb_available);

int mt6895_gpueb_power_control(unsigned int power_on)
{
	struct gpufreq_ipi_data msg = {};
	int ret;

	mutex_lock(&gpueb_power_lock);

	ret = gpueb_power_init_locked();
	if (ret)
		goto out;

	msg.cmd_id = CMD_POWER_CONTROL;
	msg.u.power_state = power_on ? POWER_ON : POWER_OFF;

	ret = mtk_ipi_send_compl(&gpueb_ipidev, gpueb_power_channel,
				 IPI_SEND_POLLING, &msg,
				 GPUFREQ_IPI_DATA_LEN, IPI_TIMEOUT_MS);
	if (ret != IPI_ACTION_DONE) {
		ret = -EIO;
		goto out;
	}

	if (gpueb_power_recv_msg.u.return_value < 0)
		ret = gpueb_power_recv_msg.u.return_value;
	else
		ret = 0;

	if (power_on && !ret) {
		/* Mirror downstream gpufreq: with dual-buck, commit the STACK OPP
		 * after power-on so the EB actually applies the working voltage/freq.
		 * OPP index 41 is 219 MHz in the MT6895 downstream OPP table, matching
		 * the normal LK boot clock.
		 */
		ret = gpueb_power_commit_locked(TARGET_STACK, 41);
		if (ret)
			pr_err("XAGA-GPUEB: CMD_COMMIT(STACK) failed: %d\n", ret);

		/* Also try committing the GPU target. Downstream normally only commits
		 * STACK on dual-buck parts, but some MT6895 firmware versions may expect
		 * a GPU-side commit before shader power-up. Keep this non-fatal so MCU
		 * boot can still proceed if the EB rejects it.
		 */
		if (!ret) {
			int gpu_ret = gpueb_power_commit_locked(TARGET_GPU, 41);

			if (gpu_ret)
				pr_err("XAGA-GPUEB: CMD_COMMIT(GPU) failed: %d\n", gpu_ret);
		}
	}

	pr_info("XAGA-GPUEB: power_control(%u) ret=%d\n", power_on, ret);

out:
	mutex_unlock(&gpueb_power_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6895_gpueb_power_control);

int mt6895_gpueb_power_on(void)
{
	return mt6895_gpueb_power_control(true);
}
EXPORT_SYMBOL_GPL(mt6895_gpueb_power_on);
