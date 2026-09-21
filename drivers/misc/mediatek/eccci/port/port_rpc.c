// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/mm_types.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/module.h>
#include <linux/sched/clock.h> /* local_clock() */
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include "ccci_config.h"
#include "ccci_common_config.h"
#include <linux/arm-smccc.h>
#include <linux/soc/mediatek/ccci_mtk_sip_svc.h>

#define TRNG_MAGIC		0x74726e67
#ifdef FEATURE_INFORM_NFC_VSIM_CHANGE
#include <mach/mt6605.h>
#endif
#ifdef FEATURE_RF_CLK_BUF
#include <mtk-clkbuf-bridge.h>
#endif

#include "ccci_core.h"
#include "ccci_auxadc.h"
#include "ccci_bm.h"
#include "ccci_modem.h"
#include "port_rpc.h"
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fcntl.h>
#include <linux/err.h>

/* PEARL-TEST：覆盖 RPC 回答的 DRDI 射频配置集索引（-1 表示沿用设备树） */
static int pearl_rf_set_idx = -1;
module_param(pearl_rf_set_idx, int, 0644);
MODULE_PARM_DESC(pearl_rf_set_idx,
	"PEARL test override for mediatek,md_drdi_rf_set_idx (-1 = use DT)");
#define MAX_QUEUE_LENGTH 16

static struct gpio_item gpio_mapping_table[] = {
	{"GPIO_FDD_Band_Support_Detection_1",
		"GPIO_FDD_BAND_SUPPORT_DETECT_1ST_PIN",},
	{"GPIO_FDD_Band_Support_Detection_2",
		"GPIO_FDD_BAND_SUPPORT_DETECT_2ND_PIN",},
	{"GPIO_FDD_Band_Support_Detection_3",
		"GPIO_FDD_BAND_SUPPORT_DETECT_3RD_PIN",},
	{"GPIO_FDD_Band_Support_Detection_4",
		"GPIO_FDD_BAND_SUPPORT_DETECT_4TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_5",
		"GPIO_FDD_BAND_SUPPORT_DETECT_5TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_6",
		"GPIO_FDD_BAND_SUPPORT_DETECT_6TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_7",
		"GPIO_FDD_BAND_SUPPORT_DETECT_7TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_8",
		"GPIO_FDD_BAND_SUPPORT_DETECT_8TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_9",
		"GPIO_FDD_BAND_SUPPORT_DETECT_9TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_A",
		"GPIO_FDD_BAND_SUPPORT_DETECT_ATH_PIN",},
	{"GPIO_RF_PWREN_RST_PIN",
		"GPIO_RF_PWREN_RST_PIN",},
};

static int get_md_gpio_val(unsigned int num)
{
	/* 不回读电平：读 GPIO 会触发 pinctrl 访问（见 get_gpio_id_from_dt 注释）。
	 * 返回 0，MODEM 仅把它当作 SIM 在位状态。
	 */
	return 0;
}

static int get_md_adc_val(__attribute__((unused))unsigned int num)
{
	int val = ccci_get_adc_val();

	return val;
}


static int get_td_eint_info(char *eint_name, unsigned int len)
{
	return -1;
}

static int get_md_adc_info(__attribute__((unused))char *adc_name,
			   __attribute__((unused))unsigned int len)
{
	int num = ccci_get_adc_num();

	CCCI_NORMAL_LOG(0, RPC, "ADC channel num:%d\n", num);
	return num;
}

static char *md_gpio_name_convert(char *gpio_name, unsigned int len)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gpio_mapping_table); i++) {
		if (!strncmp(gpio_name, gpio_mapping_table[i].gpio_name_from_md,
			len))
			return gpio_mapping_table[i].gpio_name_from_dts;
	}

	return NULL;
}

static int get_gpio_id_from_dt(struct device_node *node,
	char *gpio_name, int *md_view_id)
{
	int md_view_gpio_id = -1;
	int ret;

	/* For new API, there is a shift between AP GPIO ID and MD GPIO ID.
	 * PEARL: the DT property is <&pio 42 0>, whose index 1 is the pin number
	 * the MODEM expects (Android logs "get_num:42" for this same node).
	 *
	 * Only the device tree is read here, on purpose: requesting or reading
	 * the GPIO would go through the MT6895 pinctrl, whose registers live in
	 * the SCP shared window (0x10005000) and hang the AP while SCP is down.
	 */
	ret = of_property_read_u32_index(node, gpio_name, 1, &md_view_gpio_id);
	if (ret)
		return ret;
	*md_view_id = md_view_gpio_id;

	return md_view_gpio_id;
}

static int get_md_gpio_info(char *gpio_name,
	unsigned int len, int *md_view_gpio_id)
{
	struct device_node *node = of_find_compatible_node(NULL, NULL,
		"mediatek,gpio_usage_mapping");
	int gpio_id = -1;
	char *name;

	if (len >= 4096) {
		CCCI_NORMAL_LOG(0, RPC,
			"MD GPIO name length abnoremal(%d)\n", len);
		return gpio_id;
	}

	if (!node) {
		CCCI_NORMAL_LOG(0, RPC,
			"MD_USE_GPIO is not set in device tree,need to check?\n");
		return gpio_id;
	}

	name = md_gpio_name_convert(gpio_name, len);
	if (name) {
		gpio_id = get_gpio_id_from_dt(node, name, md_view_gpio_id);
		return gpio_id;
	}
	if (gpio_name[len-1] != 0) {
		name = kmalloc(len + 1, GFP_KERNEL);
		if (name) {
			memcpy(name, gpio_name, len);
			name[len] = 0;
			gpio_id = get_gpio_id_from_dt(node, name,
				md_view_gpio_id);
			kfree(name);
			return gpio_id;
		}
		CCCI_BOOTUP_LOG(0, RPC,
			"alloc memory fail for gpio with size:%d\n", len);
		return gpio_id;
	}
	gpio_id = get_gpio_id_from_dt(node, gpio_name, md_view_gpio_id);
	return gpio_id;
}

static int get_dram_type_clk(int *clk, int *type)
{
	return -1;
}

static struct eint_struct md_eint_struct[] = {
	/* ID of MD get, property name,  cell index read from property */
	{SIM_EINT_NUM, "interrupts", 0,},
	{SIM_EINT_DEBOUNCE, "debounce", 1,},
	{SIM_EINT_POLA, "interrupts", 1,},
	{SIM_EINT_SENS, "interrupts", 1,},
	{SIM_EINT_SOCKE, "sockettype", 1,},
	{SIM_EINT_DEDICATEDEN, "dedicated", 1,},
	{SIM_EINT_SRCPIN, "src_pin", 1,},
	{SIM_HOT_PLUG_EINT_MAX, "invalid_type", 0xFF,},
};

static struct eint_node_name md_eint_node[] = {
	{"MD1_SIM1_HOT_PLUG_EINT", 1, 1,},
	{"MD1_SIM2_HOT_PLUG_EINT", 1, 2,},
	{"MD1_SIM3_HOT_PLUG_EINT", 1, 3,},
	{"MD1_SIM4_HOT_PLUG_EINT", 1, 4,},
	/* {"MD1_SIM5_HOT_PLUG_EINT", 1, 5, }, */
	/* {"MD1_SIM6_HOT_PLUG_EINT", 1, 6, }, */
	/* {"MD1_SIM7_HOT_PLUG_EINT", 1, 7, }, */
	/* {"MD1_SIM8_HOT_PLUG_EINT", 1, 8, }, */
	/* {"MD2_SIM1_HOT_PLUG_EINT", 2, 1, }, */
	/* {"MD2_SIM2_HOT_PLUG_EINT", 2, 2, }, */
	/* {"MD2_SIM3_HOT_PLUG_EINT", 2, 3, }, */
	/* {"MD2_SIM4_HOT_PLUG_EINT", 2, 4, }, */
	/* {"MD2_SIM5_HOT_PLUG_EINT", 2, 5, }, */
	/* {"MD2_SIM6_HOT_PLUG_EINT", 2, 6, }, */
	/* {"MD2_SIM7_HOT_PLUG_EINT", 2, 7, }, */
	/* {"MD2_SIM8_HOT_PLUG_EINT", 2, 8, }, */
	{NULL,},
};

struct eint_node_struct eint_node_prop = {
	0,
	md_eint_node,
	md_eint_struct,
};

static int get_eint_attr_val(int md_id, struct device_node *node, int index)
{
	int value;
	int ret = 0, type;

	/* unit of AP eint is us, but unit of MD eint is ms.
	 * So need covertion here.
	 */
	int covert_AP_to_MD_unit = 1000;

	for (type = 0; type < SIM_HOT_PLUG_EINT_MAX; type++) {
		ret = of_property_read_u32_index(node,
			md_eint_struct[type].property,
			md_eint_struct[type].index, &value);
		if (ret != 0) {
			md_eint_struct[type].value_sim[index] =
			ERR_SIM_HOT_PLUG_QUERY_TYPE;
			CCCI_NORMAL_LOG(md_id, RPC, "%s:  not found\n",
			md_eint_struct[type].property);
			ret = ERR_SIM_HOT_PLUG_QUERY_TYPE;
			continue;
		}
		/* special case: polarity's position == sensitivity's start[ */
		if (type == SIM_EINT_POLA) {
			switch (value) {
			case IRQ_TYPE_EDGE_RISING:
			case IRQ_TYPE_EDGE_FALLING:
			case IRQ_TYPE_LEVEL_HIGH:
			case IRQ_TYPE_LEVEL_LOW:
				md_eint_struct[SIM_EINT_POLA].value_sim[index]
					= (value & 0x5) ? 1 : 0;
				/* 1/4:
				 * IRQ_TYPE_EDGE_RISING/
				 * IRQ_TYPE_LEVEL_HIGH Set 1
				 */
				md_eint_struct[SIM_EINT_SENS].value_sim[index]
					= (value & 0x3) ? 1 : 0;
				/* 1/2:
				 * IRQ_TYPE_EDGE_RISING/
				 * IRQ_TYPE_LEVEL_FALLING Set 1
				 */
				break;
			default:	/* invalid */
				md_eint_struct[SIM_EINT_POLA].value_sim[index]
					= -1;
				md_eint_struct[SIM_EINT_SENS].value_sim[index]
					= -1;
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid value, please check dtsi!\n");
				break;
			}
			type++;
		} else if (type == SIM_EINT_DEBOUNCE) {
			/* debounce time should divide by 1000 due
			 * to different unit in AP and MD.
			 */
			md_eint_struct[type].value_sim[index] =
				value/covert_AP_to_MD_unit;
		} else
			md_eint_struct[type].value_sim[index] = value;
	}
	return ret;
}

void get_dtsi_eint_node(int md_id)
{
	static int init; /*default is 0*/
	int i;
	struct device_node *node = NULL;

	if (init)
		return;
	init = 1;
	for (i = 0; i < MD_SIM_MAX; i++) {
		if (eint_node_prop.name[i].node_name == NULL) {
			CCCI_INIT_LOG(md_id, RPC, "node %d is NULL\n", i);
			break;
		}
		node = of_find_node_by_name(NULL,
			eint_node_prop.name[i].node_name);
		if (node != NULL) {
			eint_node_prop.ExistFlag |= (1U << i);
			get_eint_attr_val(md_id, node, i);
		} else {
			CCCI_INIT_LOG(md_id, RPC, "%s: node %d no found\n",
				     eint_node_prop.name[i].node_name, i);
		}
	}
}

int get_eint_attr_DTSVal(int md_id, const char *name, unsigned int name_len,
			unsigned int type, char *result, unsigned int *len)
{
	int i, sim_value;
	int *sim_info = (int *)result;

	if ((name == NULL) || (result == NULL) || (len == NULL))
		return ERR_SIM_HOT_PLUG_NULL_POINTER;
	if (type >= SIM_HOT_PLUG_EINT_MAX)
		return ERR_SIM_HOT_PLUG_QUERY_TYPE;

	for (i = 0; i < MD_SIM_MAX; i++) {
		if ((eint_node_prop.ExistFlag & (1U << i)) == 0)
			continue;
		if (!(strncmp(name,
			eint_node_prop.name[i].node_name, name_len))) {
			sim_value =
			eint_node_prop.eint_value[type].value_sim[i];
			*len = sizeof(sim_value);
			memcpy(sim_info, &sim_value, *len);
			CCCI_BOOTUP_LOG(md_id, RPC,
			"md_eint:%s, sizeof: %d, sim_info: %d, %d\n",
			eint_node_prop.eint_value[type].property,
			*len, *sim_info,
			eint_node_prop.eint_value[type].value_sim[i]);
			if (sim_value >= 0)
				return 0;
		}
	}
	return ERR_SIM_HOT_PLUG_QUERY_STRING;
}

static int get_eint_attr(int md_id, char *name, unsigned int name_len,
			unsigned int type, char *result, unsigned int *len)
{
	return get_eint_attr_DTSVal(md_id, name, name_len, type, result, len);
}

static void get_md_dtsi_val(struct ccci_rpc_md_dtsi_input *input,
	struct ccci_rpc_md_dtsi_output *output)
{
	int ret = -1;
	int value = 0;
	struct device_node *node =
	of_find_compatible_node(NULL, NULL, "mediatek,md_attr_node");

	/* PEARL-TEST：先看测试覆盖参数 */
	if (pearl_rf_set_idx >= 0 &&
	    strncmp(input->strName, "mediatek,md_drdi_rf_set_idx",
		    strlen("mediatek,md_drdi_rf_set_idx")) == 0) {
		output->retValue = (unsigned int)pearl_rf_set_idx;
		CCCI_ERROR_LOG(-1, RPC, "PEARL-TEST: rf_set_idx -> %d\n",
			pearl_rf_set_idx);
		return;
	}

	if (node == NULL) {
		CCCI_INIT_LOG(-1, RPC, "%s: No node: %s\n", __func__,
			input->strName);
		CCCI_NORMAL_LOG(-1, RPC, "%s: No node: %s\n", __func__,
			input->strName);
		return;
	}

	switch (input->req) {
	case RPC_REQ_PROP_VALUE:
		ret = of_property_read_u32(node, input->strName, &value);
		if (ret == 0)
			output->retValue = value;
		break;
	}
	CCCI_INIT_LOG(-1, RPC, "%s %d, %s -- 0x%x\n", __func__,
		input->req, input->strName, output->retValue);
	CCCI_NORMAL_LOG(-1, RPC, "%s %d, %s -- 0x%x\n", __func__,
		input->req, input->strName, output->retValue);
}

static void get_md_dtsi_debug(void)
{
	struct ccci_rpc_md_dtsi_input input;
	struct ccci_rpc_md_dtsi_output output;
	int ret;

	input.req = RPC_REQ_PROP_VALUE;
	output.retValue = 0;
	ret = snprintf(input.strName, sizeof(input.strName), "%s",
		"mediatek,md_drdi_rf_set_idx");
	if (ret <= 0 || ret >= sizeof(input.strName)) {
		CCCI_ERROR_LOG(-1, RPC, "%s:snprintf input.strName fail\n",
			__func__);
		return;
	}
	get_md_dtsi_val(&input, &output);
}

static void ccci_rpc_get_gpio_adc(struct ccci_rpc_gpio_adc_intput *input,
	struct ccci_rpc_gpio_adc_output *output)
{
	int num;
	unsigned int val, i, md_val = -1;

	if ((input->reqMask & (RPC_REQ_GPIO_PIN | RPC_REQ_GPIO_VALUE)) ==
		(RPC_REQ_GPIO_PIN | RPC_REQ_GPIO_VALUE)) {
		for (i = 0; i < GPIO_MAX_COUNT; i++) {
			if (input->gpioValidPinMask & (1 << i)) {
				num = get_md_gpio_info(input->gpioPinName[i],
						strlen(input->gpioPinName[i]),
						&md_val);
				if (num >= 0) {
					output->gpioPinNum[i] = md_val;
					val = get_md_gpio_val(num);
					output->gpioPinValue[i] = val;
				}
			}
		}
	} else {
		if (input->reqMask & RPC_REQ_GPIO_PIN) {
			for (i = 0; i < GPIO_MAX_COUNT; i++) {
				if (input->gpioValidPinMask & (1 << i)) {
					num = get_md_gpio_info(
					input->gpioPinName[i],
					strlen(input->gpioPinName[i]), &md_val);
					if (num >= 0)
						output->gpioPinNum[i] = md_val;
				}
			}
		}
		if (input->reqMask & RPC_REQ_GPIO_VALUE) {
			for (i = 0; i < GPIO_MAX_COUNT; i++) {
				if (input->gpioValidPinMask & (1 << i)) {
					val = get_md_gpio_val(
					input->gpioPinNum[i]);
					output->gpioPinValue[i] = val;
				}
			}
		}
	}
	if ((input->reqMask & (RPC_REQ_ADC_PIN | RPC_REQ_ADC_VALUE)) ==
		(RPC_REQ_ADC_PIN | RPC_REQ_ADC_VALUE)) {
		num = get_md_adc_info(input->adcChName,
				strlen(input->adcChName));

		if (num >= 0) {
			output->adcChNum = num;
			output->adcChMeasSum = 0;
			for (i = 0; i < input->adcChMeasCount; i++) {
				val = get_md_adc_val(num);
				output->adcChMeasSum += val;
			}
			CCCI_NORMAL_LOG(0, RPC,
					"%s, reqMask:%d, adcChmeasCount:%u, adcChMeasSum:%u\n",
					__func__, input->reqMask, i, output->adcChMeasSum);
		}
	} else {
		if (input->reqMask & RPC_REQ_ADC_PIN) {
			num = get_md_adc_info(input->adcChName,
					strlen(input->adcChName));
			if (num >= 0)
				output->adcChNum = num;
		}
		if (input->reqMask & RPC_REQ_ADC_VALUE) {
			output->adcChMeasSum = 0;
			for (i = 0; i < input->adcChMeasCount; i++) {
				val = get_md_adc_val(input->adcChNum);
				output->adcChMeasSum += val;
			}
			CCCI_NORMAL_LOG(0, RPC,
					"%s, reqMask:%d, adcChmeasCount:%u, adcChMeasSum:%u\n",
					__func__, input->reqMask, i, output->adcChMeasSum);
		}
	}
}

static void ccci_rpc_get_gpio_adc_v2(struct ccci_rpc_gpio_adc_intput_v2 *input,
	struct ccci_rpc_gpio_adc_output_v2 *output)
{
	int num, md_val = -1;
	unsigned int val, i;

	if ((input->reqMask & (RPC_REQ_GPIO_PIN | RPC_REQ_GPIO_VALUE)) ==
		(RPC_REQ_GPIO_PIN | RPC_REQ_GPIO_VALUE)) {
		for (i = 0; i < GPIO_MAX_COUNT_V2; i++) {
			if (input->gpioValidPinMask & (1 << i)) {
				num = get_md_gpio_info(input->gpioPinName[i],
						strlen(input->gpioPinName[i]),
						&md_val);
				if (num >= 0) {
					output->gpioPinNum[i] = md_val;
					val = get_md_gpio_val(num);
					output->gpioPinValue[i] = val;
				}
			}
		}
	} else {
		if (input->reqMask & RPC_REQ_GPIO_PIN) {
			for (i = 0; i < GPIO_MAX_COUNT_V2; i++) {
				if (input->gpioValidPinMask & (1 << i)) {
					num = get_md_gpio_info(
						input->gpioPinName[i],
						strlen(input->gpioPinName[i]),
						&md_val);
					if (num >= 0)
						output->gpioPinNum[i] = md_val;
				}
			}
		}
		if (input->reqMask & RPC_REQ_GPIO_VALUE) {
			for (i = 0; i < GPIO_MAX_COUNT_V2; i++) {
				if (input->gpioValidPinMask & (1 << i)) {
					val = get_md_gpio_val(
							input->gpioPinNum[i]);
					output->gpioPinValue[i] = val;
				}
			}
		}
	}
	if ((input->reqMask & (RPC_REQ_ADC_PIN | RPC_REQ_ADC_VALUE)) ==
		(RPC_REQ_ADC_PIN | RPC_REQ_ADC_VALUE)) {
		num = get_md_adc_info(input->adcChName,
				strlen(input->adcChName));
		if (num >= 0) {
			output->adcChNum = num;
			output->adcChMeasSum = 0;
			for (i = 0; i < input->adcChMeasCount; i++) {
				val = get_md_adc_val(num);
				output->adcChMeasSum += val;
			}
			CCCI_NORMAL_LOG(0, RPC,
					"%s, reqMask:%d, adcChmeasCount:%u, adcChMeasSum:%u\n",
					__func__, input->reqMask, i, output->adcChMeasSum);
		}
	} else {
		if (input->reqMask & RPC_REQ_ADC_PIN) {
			num = get_md_adc_info(input->adcChName,
					strlen(input->adcChName));
			if (num >= 0)
				output->adcChNum = num;
		}
		if (input->reqMask & RPC_REQ_ADC_VALUE) {
			output->adcChMeasSum = 0;
			for (i = 0; i < input->adcChMeasCount; i++) {
				val = get_md_adc_val(input->adcChNum);
				output->adcChMeasSum += val;
			}
			CCCI_NORMAL_LOG(0, RPC,
					"%s, reqMask:%d, adcChmeasCount:%u, adcChMeasSum:%u\n",
					__func__, input->reqMask, i, output->adcChMeasSum);
		}
	}
}

static int ccci_rpc_remap_queue(int md_id, struct ccci_rpc_queue_mapping *remap)
{
	struct port_t *port;

	port = port_get_by_minor(md_id, remap->net_if + CCCI_NET_MINOR_BASE);

	if (!port) {
		CCCI_ERROR_LOG(md_id, RPC, "can't find ccmni for netif: %d\n",
			remap->net_if);
		return -1;
	}

	if (remap->lhif_q == LHIF_HWQ_AP_UL_Q0) {
		/*normal queue*/
		port->txq_index = 0;
		port->txq_exp_index = 0xF0 | 0x1;
		CCCI_NORMAL_LOG(md_id, RPC, "remap port %s Tx to cldma%d\n",
			port->name, port->txq_index);
	} else if (remap->lhif_q == LHIF_HWQ_AP_UL_Q1) {
		/*IMS queue*/
		port->txq_index = 3;
		port->txq_exp_index = 0xF0 | 0x3;
		CCCI_NORMAL_LOG(md_id, RPC, "remap port %s Tx to cldma%d\n",
			port->name, port->txq_index);
	} else
		CCCI_ERROR_LOG(md_id, RPC, "invalid remap for q%d\n",
			remap->lhif_q);

	return 0;
}

/* ===== PEARL: AMMS DRDI control (kernel-side replacement for ccci_rpcd) =====
 *
 * 协议由 vendor/bin/ccci_rpcd 反汇编 + yuechu 真机字节级验证还原，
 * 详见 E:\pearl\notes\rpcd\PROTOCOL.md 与 amms_drdi_protocol.h。
 *
 * 请求 (op_id=0x4014, para_num=1, para[0].len=188)：
 *   +0x00 u8 cmd          1=INIT / 2=DRDI_COPY
 *   +0x01 u8 seq_id       回显到应答
 *   +0x04 u8 ver          INIT 必须==3 ; COPY 必须==1
 *   +0x05 u8 set_total_num <= 15
 *   +0x08 INIT 表: {u32 offset; u32 len;} stride 8   (相对 md1drdi 数据区)
 *   +0x08 COPY 表: {u32 src; u32 dst; u32 len;} stride 12
 *                 src 相对 md1drdi 数据区，dst 是 64KiB DRDI smem 内偏移
 *
 * 应答 (op_id=0xFFFF4014, para_num=2, 共 44 字节)：
 *   para[0] = {u32 len=4, u32 ret_code(0/0xFFFFFFFF)}
 *   para[1] = {u32 len=8, 8 字节状态}
 *   状态: {stats, seq_id, rsv[2], ver, copystat, drdiinfostat, rsv}
 */
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/vmalloc.h>

#define PEARL_AMMS_MAX_SET	15
#define PEARL_AMMS_REQ_SIZE	188	/* 8 + 15*12，内核硬校验 0xBC */
#define PEARL_AMMS_CMD_INIT	1
#define PEARL_AMMS_CMD_COPY	2
#define PEARL_AMMS_INIT_VER	3
#define PEARL_AMMS_COPY_VER	1
#define PEARL_AMMS_MAX_COPY_LEN	0x10000
#define PEARL_AMMS_MD_NUM	2

#define PEARL_MD1IMG_PATH	"/dev/disk/by-partlabel/md1img_a"
#define PEARL_SEG_MAGIC		0x58881688
#define PEARL_SEG_HDR_LEN	0x200	/* 段头 hdrlen 默认值，数据区在其后 */
#define PEARL_DRDI_SEG_NAME	"md1drdi"
#define PEARL_SCAN_CHUNK		(4 * 1024 * 1024)
#define PEARL_SCAN_BLOCKS	40

struct pearl_amms_req {
	u8 cmd;
	u8 seq_id;
	u8 rsv0[2];
	u8 ver;
	u8 set_total_num;
	u8 rsv1[2];
	u8 tbl[180];
} __packed;

struct pearl_amms_rsp {
	u8 stats;		/* 0 = 成功, 0xFF = 失败 */
	u8 seq_id;		/* = req.seq_id */
	u8 rsv2[2];
	u8 ver;
	u8 copystat;
	u8 drdiinfostat;
	u8 rsv3;
} __packed;

struct pearl_amms_set_init {
	u32 off;
	u32 len;
} __packed;

struct pearl_amms_set_copy {
	u32 src;
	u32 dst;
	u32 len;
} __packed;

static unsigned int pearl_amms_dump_req = 1;
module_param(pearl_amms_dump_req, uint, 0644);
MODULE_PARM_DESC(pearl_amms_dump_req, "PEARL: dump every AMMS DRDI request");

static unsigned int pearl_amms_do_copy = 1;
module_param(pearl_amms_do_copy, uint, 0644);
MODULE_PARM_DESC(pearl_amms_do_copy, "PEARL: perform the AMMS DRDI data copy");

static unsigned int pearl_amms_fail_ok = 0;
module_param(pearl_amms_fail_ok, uint, 0644);
MODULE_PARM_DESC(pearl_amms_fail_ok,
	"PEARL: force error reply (for A/B testing the reply path)");

/* 从 INIT 请求留下的状态（ccci_rpcd 用全局变量保存，COPY 表项数取自它） */
static unsigned int pearl_amms_set_total[PEARL_AMMS_MD_NUM];
static int pearl_amms_copy_done[PEARL_AMMS_MD_NUM];
static unsigned int pearl_amms_req_cnt[PEARL_AMMS_MD_NUM];

/* md1drdi 段数据区（段头 +0x200 起），请求里的 offset/src 以此为基准 */
static void *pearl_drdi_data;
static unsigned int pearl_drdi_len;
static unsigned int pearl_drdi_seg_off;

static u32 pearl_le32(const unsigned char *p)
{
	return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
		((u32)p[3] << 24);
}

/* 从 md1img 分区里找出 md1drdi 段并读入内存（insmod 时调用一次） */
static int pearl_drdi_load_image(void)
{
	struct file *f;
	unsigned char *buf;
	loff_t pos;
	unsigned int i, blk, got, seg_len = 0;
	int ret = 0;

	if (pearl_drdi_data)
		return 0;

	f = filp_open(PEARL_MD1IMG_PATH, O_RDONLY | O_LARGEFILE, 0);
	if (IS_ERR(f)) {
		ret = PTR_ERR(f);
		pr_err("PEARL-AMMS: open %s fail %d\n", PEARL_MD1IMG_PATH, ret);
		return ret;
	}
	buf = vmalloc(PEARL_SCAN_CHUNK + PEARL_SEG_HDR_LEN);
	if (!buf) {
		ret = -ENOMEM;
		goto out_close;
	}
	for (blk = 0; blk < PEARL_SCAN_BLOCKS && !seg_len; blk++) {
		pos = (loff_t)blk * PEARL_SCAN_CHUNK;
		got = kernel_read(f, buf, PEARL_SCAN_CHUNK, &pos);
		if ((int)got <= 0)
			break;
		for (i = 0; i + 16 <= got; i += 8) {
			if (pearl_le32(buf + i) != PEARL_SEG_MAGIC)
				continue;
			if (memcmp(buf + i + 8, PEARL_DRDI_SEG_NAME, 8))
				continue;
			seg_len = pearl_le32(buf + i + 4);
			pearl_drdi_seg_off = (unsigned int)pos - got + i;
			break;
		}
	}
	if (!seg_len) {
		pr_err("PEARL-AMMS: %s segment not found\n", PEARL_DRDI_SEG_NAME);
		ret = -ENOENT;
		goto out_free;
	}
	pearl_drdi_data = vmalloc(seg_len);
	if (!pearl_drdi_data) {
		ret = -ENOMEM;
		goto out_free;
	}
	pos = (loff_t)pearl_drdi_seg_off + PEARL_SEG_HDR_LEN;
	got = kernel_read(f, pearl_drdi_data, seg_len, &pos);
	if ((int)got != (int)seg_len) {
		pr_err("PEARL-AMMS: read drdi seg short %u/%u\n", got, seg_len);
		vfree(pearl_drdi_data);
		pearl_drdi_data = NULL;
		ret = -EIO;
		goto out_free;
	}
	pearl_drdi_len = seg_len;
	pr_info("PEARL-AMMS: md1drdi loaded seg_off=0x%x data=0x%x len=0x%x\n",
		pearl_drdi_seg_off, pearl_drdi_seg_off + PEARL_SEG_HDR_LEN,
		pearl_drdi_len);
out_free:
	vfree(buf);
out_close:
	filp_close(f, NULL);
	return ret;
}

/* ===== PEARL: NVRAM cache 共享内存填充 =====
 * Mobian 没有 Android 的 nvram 服务，SMEM_USER_MD_NVRAM_CACHE（AP 视图
 * 0x8a180000，1.5MB）实测全为 0。MODEM 的 RF 校准数据来自 NVRAM，
 * 读到全零就会在 mml1_rf_error_check 断言，所以这里在 MODEM 读它之前
 * （AMMS init 请求时刻）把 nvram 分区内容灌进该共享区。
 * 源、偏移、长度都可配，便于实验。
 */
#define PEARL_NVRAM_SRC_DEFAULT	"/dev/disk/by-partlabel/nvram"

static char *pearl_nvram_src = PEARL_NVRAM_SRC_DEFAULT;
module_param(pearl_nvram_src, charp, 0444);
MODULE_PARM_DESC(pearl_nvram_src, "PEARL: NVRAM source for the MD cache region");

static unsigned int pearl_nvram_skip;
module_param(pearl_nvram_skip, uint, 0444);
MODULE_PARM_DESC(pearl_nvram_skip, "PEARL: source offset");

static unsigned int pearl_nvram_len;
module_param(pearl_nvram_len, uint, 0444);
MODULE_PARM_DESC(pearl_nvram_len, "PEARL: bytes to copy (0 = whole region)");

static unsigned int pearl_nvram_fill = 1;
module_param(pearl_nvram_fill, uint, 0644);
MODULE_PARM_DESC(pearl_nvram_fill, "PEARL: fill NVRAM cache region at AMMS init");

static void *pearl_nvram_data;
static unsigned int pearl_nvram_data_len;
static unsigned int pearl_nvram_done[PEARL_AMMS_MD_NUM];

/* insmod 时把 NVRAM 源读进内存 */
static int pearl_nvram_load(void)
{
	struct file *f;
	loff_t pos;
	unsigned int want = 0x200000;	/* 先读 2MB，够覆盖 1.5MB 的 cache 区 */
	int got;

	if (pearl_nvram_data)
		return 0;
	f = filp_open(pearl_nvram_src, O_RDONLY | O_LARGEFILE, 0);
	if (IS_ERR(f)) {
		pr_err("PEARL-AMMS: open nvram src %s fail %ld\n",
			pearl_nvram_src, PTR_ERR(f));
		return PTR_ERR(f);
	}
	pearl_nvram_data = vmalloc(want);
	if (!pearl_nvram_data) {
		filp_close(f, NULL);
		return -ENOMEM;
	}
	pos = pearl_nvram_skip;
	got = kernel_read(f, pearl_nvram_data, want, &pos);
	filp_close(f, NULL);
	if (got <= 0) {
		pr_err("PEARL-AMMS: read nvram src fail %d\n", got);
		vfree(pearl_nvram_data);
		pearl_nvram_data = NULL;
		return -EIO;
	}
	pearl_nvram_data_len = got;
	pr_info("PEARL-AMMS: nvram src %s skip=0x%x read=0x%x\n",
		pearl_nvram_src, pearl_nvram_skip, pearl_nvram_data_len);
	return 0;
}

/* MODEM 即将读 NVRAM 之前，把数据写进它的 cache 共享区。
 * mark=false 用于 insmod 时先填一次（此时可能还会被 MD 启动流程清掉）。
 */
static void pearl_nvram_fill_cache(int md_id, int mark)
{
	struct ccci_smem_region *r;
	void __iomem *dst;
	bool own = false;
	unsigned int len, i, nz_before = 0, nz_after = 0;
	u8 *rb;

	if (!pearl_nvram_fill)
		return;
	if (mark && pearl_nvram_done[md_id & 1])
		return;
	if (!pearl_nvram_data && pearl_nvram_load())
		return;
	r = ccci_md_get_smem_by_user_id(md_id, SMEM_USER_MD_NVRAM_CACHE);
	if (!r || !r->size) {
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS: no NVRAM cache region (%p)\n", r);
		return;
	}
	len = pearl_nvram_len ? pearl_nvram_len : r->size;
	if (len > r->size)
		len = r->size;
	if (len > pearl_nvram_data_len)
		len = pearl_nvram_data_len;

	dst = r->base_ap_view_vir;
	if (!dst) {
		dst = ioremap_wc(r->base_ap_view_phy, r->size);
		own = true;
	}
	if (!dst) {
		CCCI_ERROR_LOG(md_id, RPC, "PEARL-AMMS: map nvram cache fail\n");
		return;
	}
	rb = vmalloc(len);
	if (rb) {
		/* 填充前先看当前内容：判断 insmod 时填的是否已被清掉 */
		memcpy_fromio(rb, dst, len);
		for (i = 0; i < len; i++)
			if (rb[i])
				nz_before++;
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS: NVRAM cache before fill: len=0x%x nonzero=%u first=%*ph\n",
			len, nz_before, 16, rb);
	}
	memcpy_toio(dst, pearl_nvram_data, len);
	if (rb) {
		memcpy_fromio(rb, dst, len);
		for (i = 0; i < len; i++)
			if (rb[i])
				nz_after++;
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS: NVRAM cache filled%s len=0x%x nonzero=%u first=%*ph\n",
			mark ? "" : "(insmod)", len, nz_after, 16, rb);
		vfree(rb);
	}
	if (own)
		iounmap(dst);
	if (mark)
		pearl_nvram_done[md_id & 1] = 1;
}

/*
 * PEARL: Android 的 ccci_mdinit 是「等 NVRAM 就绪 -> 再 DO_START_MD」，
 * 我们以前是「先启动 -> AMMS init 时才填 NVRAM」，基带启动早期可能读到 0。
 * 这里把准备动作提前到启动命令之前（懒加载兜底保留，分区没就绪也不会漏）。
 */
void pearl_prepare_before_md_start(unsigned char md_id)
{
	int ret;

	ret = pearl_drdi_load_image();
	if (!pearl_drdi_data)
		pr_err("PEARL-MD-START: md1drdi not ready yet (ret=%d), lazy path will retry\n",
		       ret);

	pearl_nvram_fill_cache(md_id, 2);
	pr_err("PEARL-MD-START: nvram cache prepared before MD start (len=%u)\n",
	       pearl_nvram_data_len);
}
EXPORT_SYMBOL(pearl_prepare_before_md_start);

/*
 * PEARL: FS(ccci_fs, ch14/ch15) 服务端
 *
 * Android 上这条通道由用户态 ccci_fsd 服务；新版 MTK 已把它并进 ccci_mdinit
 * （实测 yuechu：/proc/<ccci_mdinit>/fd/10 -> /dev/ccci_fs，ccci_fsd 根本没被启动），
 * 基带照样能 boot 到 ready。Mobian 侧没有任何进程打开 /dev/ccci_fs，于是基带在
 * HS1 之后发的 op=0x1001(FS_CCCI_Open) 请求永远等不到回复，43s 后
 * MD_BOOT_HS2_FAIL —— 这是早期异常(A 形态)消失之后剩下的唯一失败原因。
 *
 * 报文格式（yuechu 真机 strace 反解，详见 notes/rpcd/FS_PROTOCOL.md）：
 *   [struct ccci_header 16B][u32 op][u32 nblocks]
 *   nblocks * { u32 len; u8 data[len]，按 4 字节对齐 }
 * 回复：channel 改成 CCCI_FS_TX、op 或上 0xFFFF0000、seq 原样回填。
 *
 * 各 op 的块结构（yuechu + pearl 实测）：
 *   0x1001 Open        req {len: path(UTF-16LE)}{4: mode}   rep {4: handle}
 *   0x1002 Seek        req {4: handle}{4: off}{4: whence}   rep {4: new_pos}
 *   0x1004 Write       req {4: handle}{len: data}{4: off}   rep {4: status}
 *   0x1005 Close       req {4: handle}                      rep {4: status}
 *   0x1009 GetFileSize req {4: handle}                      rep {4: status}{4: size}
 *
 * 这是"内存里的迷你文件系统"：文件按名字持久（跨 open/close），Write 真存数据。
 * 基带因此能像在 Android 上一样追加写 nv_boot_trace，而我们用
 * /proc/pearl_fs 就能把基带自己写的 boot trace 读出来。
 * 实测教训：早期版本 Close 时清掉 size，基带重开后再 Seek(END) 拿到 0，
 * 就会用同一个 seq 疯狂重发 Seek，最后在 dev_fs.c:224 断言。
 */
#define PEARL_FS_MAX_MSG	4096
#define PEARL_FS_MAX_BLK	8
#define PEARL_FS_MAX_FILE	24
#define PEARL_FS_MAX_HANDLE	16
#define PEARL_FS_MAX_CAP	(256 * 1024)
#define PEARL_FS_OP_OPEN	0x1001
#define PEARL_FS_OP_SEEK	0x1002
#define PEARL_FS_OP_READ	0x1003
#define PEARL_FS_OP_WRITE	0x1004
#define PEARL_FS_OP_CLOSE	0x1005
#define PEARL_FS_OP_CLOSE_ALL	0x1006
#define PEARL_FS_OP_FILE_SIZE	0x1009
#define PEARL_FS_OP_CMPT_READ	0x1022	/* 整文件读（带状态位图）*/
#define PEARL_FS_OP_FIND_FIRST	0x1012
#define PEARL_FS_OP_FIND_NEXT	0x1013
#define PEARL_FS_OP_FIND_CLOSE	0x1014

/* 0 = 不响应（A/B 对照用）；1/2 = 预留的降级模式；>=2 正常应答 */
static int pearl_fs_mode = 2;
module_param(pearl_fs_mode, int, 0644);
MODULE_PARM_DESC(pearl_fs_mode, "PEARL FS(ccci_fs): 0=off, 2=normal");

static atomic_t pearl_fs_msg_cnt = ATOMIC_INIT(0);
static DEFINE_MUTEX(pearl_fs_lock);

struct pearl_fs_file {
	int used;
	char name[96];
	unsigned char *data;
	unsigned int size;
	unsigned int cap;
};

struct pearl_fs_handle {
	int used;
	int file;
	unsigned int pos;
	struct file *fp;	/* 真实文件句柄（/mnt/nvdata 下） */
};

static struct pearl_fs_file pearl_fs_files[PEARL_FS_MAX_FILE];
static struct pearl_fs_handle pearl_fs_handles[PEARL_FS_MAX_HANDLE];

static struct pearl_fs_job {
	struct work_struct work;
	unsigned char md_id;
	unsigned int len;
	unsigned char data[];
} *pearl_fs_job;

static int pearl_fs_file_find(const char *name)
{
	int i;

	for (i = 0; i < PEARL_FS_MAX_FILE; i++)
		if (pearl_fs_files[i].used &&
		    strcmp(pearl_fs_files[i].name, name) == 0)
			return i;
	return -1;
}

static int pearl_fs_file_new(const char *name)
{
	int i;

	for (i = 0; i < PEARL_FS_MAX_FILE; i++) {
		if (!pearl_fs_files[i].used) {
			pearl_fs_files[i].used = 1;
			strscpy(pearl_fs_files[i].name, name,
				sizeof(pearl_fs_files[i].name));
			pearl_fs_files[i].data = NULL;
			pearl_fs_files[i].size = 0;
			pearl_fs_files[i].cap = 0;
			return i;
		}
	}
	return -1;
}

static int pearl_fs_file_reserve(struct pearl_fs_file *f, unsigned int need)
{
	unsigned int cap;
	unsigned char *p;

	if (need <= f->cap)
		return 0;
	if (need > PEARL_FS_MAX_CAP)
		return -1;
	for (cap = 4096; cap < need; cap <<= 1)
		;
	p = kmalloc(cap, GFP_KERNEL);
	if (p == NULL)
		return -1;
	if (f->data != NULL) {
		memcpy(p, f->data, f->size);
		kfree(f->data);
	}
	f->data = p;
	f->cap = cap;
	return 0;
}

static int pearl_fs_handle_alloc(int file)
{
	int i;

	for (i = 0; i < PEARL_FS_MAX_HANDLE; i++) {
		if (!pearl_fs_handles[i].used) {
			pearl_fs_handles[i].used = 1;
			pearl_fs_handles[i].file = file;
			pearl_fs_handles[i].pos = 0;
			return i + 1;
		}
	}
	return 0;
}

/* ---- /proc/pearl_fs：把基带写进来的文件读出来 ---- */
static int pearl_fs_proc_show(struct seq_file *m, void *v)
{
	int i, n;

	mutex_lock(&pearl_fs_lock);
	for (i = 0; i < PEARL_FS_MAX_FILE; i++) {
		if (!pearl_fs_files[i].used)
			continue;
		seq_printf(m, "=== [%d] %s  size=%u cap=%u ===\n", i,
			pearl_fs_files[i].name, pearl_fs_files[i].size,
			pearl_fs_files[i].cap);
		if (pearl_fs_files[i].data != NULL) {
			n = pearl_fs_files[i].size;
			if (n > 65536)
				n = 65536;
			/* 按文本输出，不可打印字符替换成 '.' */
			{
				char *buf = kmalloc(n + 1, GFP_KERNEL);
				int k;

				if (buf != NULL) {
					for (k = 0; k < n; k++) {
						unsigned char c =
							pearl_fs_files[i].data[k];

						buf[k] = (c >= 0x20 && c < 0x7f)
							? (char)c : '.';
					}
					buf[n] = 0;
					seq_puts(m, buf);
					kfree(buf);
				}
			}
			seq_puts(m, "\n");
		}
	}
	mutex_unlock(&pearl_fs_lock);
	return 0;
}

static int pearl_fs_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pearl_fs_proc_show, NULL);
}

static const struct proc_ops pearl_fs_proc_fops = {
	.proc_open = pearl_fs_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void pearl_fs_proc_init(void)
{
	proc_create("pearl_fs", 0444, NULL, &pearl_fs_proc_fops);
}

/* ---- 报文构造 ---- */
static unsigned int pearl_fs_put_block(unsigned char *dst, unsigned int off,
	const void *data, unsigned int len)
{
	unsigned int alen = (len + 3) & ~3U;

	*(unsigned int *)(dst + off) = len;
	off += sizeof(unsigned int);
	memcpy(dst + off, data, len);
	if (alen != len)
		memset(dst + off + len, 0, alen - len);
	return off + alen;
}

static void pearl_fs_wcs2cs(const unsigned char *wcs, unsigned int len,
	char *out, unsigned int out_len)
{
	unsigned int i;

	out[0] = 0;
	for (i = 0; i + 1 < len && (i / 2) + 1 < out_len; i += 2) {
		unsigned int c = wcs[i] | (wcs[i + 1] << 8);

		if (c == 0)
			break;
		out[i / 2] = (c < 0x80) ? (char)c : '?';
	}
	out[(i / 2) < out_len ? (i / 2) : (out_len - 1)] = 0;
}

static void pearl_fs_send(unsigned char md_id, unsigned char *msg,
	unsigned int len)
{
	struct port_t *port;
	struct sk_buff *skb;
	void *ptr;

	port = port_get_by_channel(md_id, CCCI_FS_TX);
	if (port == NULL) {
		pr_err("PEARL-FS: cannot find CCCI_FS_TX port\n");
		return;
	}
	skb = ccci_alloc_skb(len, 1, 1);
	if (skb == NULL) {
		pr_err("PEARL-FS: alloc skb(%u) fail\n", len);
		return;
	}
	ptr = skb_put(skb, len);
	memcpy(ptr, msg, len);
	if (port_send_skb_to_md(port, skb, 1) != 0) {
		pr_err("PEARL-FS: send reply fail\n");
		ccci_free_skb(skb);
	}
}


/*
 * 基带路径 -> Linux 路径。
 *   Z:\NVRAM\CALIBRAT\ML09_001 -> /mnt/nvdata/md/NVRAM/CALIBRAT/ML09_001
 * X:\ 的真实根还没完全确定，所以按候选根依次探测，谁能打开就用谁。
 */
static const char *pearl_fs_roots[] = {
	/* PEARL-FS-XROOT: X: 的真实根。
	 * 健康 yuechu(HyperOS) 日志实测：
	 *   ccci_fsd: CreateDir: [error]fail create Dir /mnt/vendor/nvcfg/mdota: 17
	 * 基带 "X:\..." 落到 AP 侧就是 /mnt/vendor/nvcfg/。
	 * 这里 nvcfg 分区在 Mobian 上挂到 /mnt/nvcfg/。
	 */
	"/mnt/nvcfg/",
	"/mnt/nvdata/md/",
	"/mnt/nvdata/",
	"/mnt/nvdata/md_cmn/",
};

/* PEARL-FS-NVCFG: 基带要读的 "X:\nv_config" 在 AP 侧并不存在实体文件
 * （实测它是我们 O_CREAT 出来的空文件，CMPTREAD 读到 0 字节后基带无限重试）。
 * MTK 的 NVRAM 索引表就是 /mnt/nvdata/AllMap（24272 字节，与 nvram 分区头等长），
 * 那正是"nv config"的内容，所以这里直接映射过去。
 */
static const struct {
	const char *modem_path;
	const char *linux_path;
} pearl_fs_path_map[] = {
	/* PEARL-FS-NVCFG-OFF: 原先把 X:\nv_config 映射到 /mnt/nvdata/AllMap，
	 * 但健康 yuechu 日志显示这个 open 本来就失败、是个探测：
	 *   ccci_fsd: O: X:/nv_config, flag 0x500, ret -9
	 * 让它失败，基带才会走对分支（别处也会读 AllMap 真正的消费者另有其人）。
	 * 这里保留一条必然失败的表项，避免零长数组。
	 */
	{ "X:\\nv_config", "/nonexistent/pearl_fs_no_nv_config" },
};

static int pearl_fs_map_path(const char *mpath, char *out, unsigned int outlen)
{
	const char *rest = mpath;
	unsigned int i, k = 0;
	char cand[256];
	int r;

	for (i = 0; i < ARRAY_SIZE(pearl_fs_path_map); i++) {
		if (strcmp(mpath, pearl_fs_path_map[i].modem_path) == 0) {
			struct file *f = filp_open(pearl_fs_path_map[i].linux_path,
						   O_RDONLY, 0);

			if (!IS_ERR(f)) {
				filp_close(f, NULL);
				strscpy(out, pearl_fs_path_map[i].linux_path, outlen);
				return 0;
			}
			pr_err("PEARL-FS: nv_config map target missing: %s\n",
			       pearl_fs_path_map[i].linux_path);
		}
	}

	if (mpath[0] != 0 && mpath[1] == ':') {
		rest = mpath + 2;
		if (rest[0] == '\\' || rest[0] == '/')
			rest++;
	}
	for (r = 0; r < ARRAY_SIZE(pearl_fs_roots); r++) {
		k = 0;
		k += scnprintf(cand + k, sizeof(cand) - k, "%s",
			pearl_fs_roots[r]);
		{
			const char *p = rest;

			while (*p != 0 && k + 1 < sizeof(cand)) {
				cand[k++] = (*p == '\\') ? '/' : *p;
				p++;
			}
		}
		cand[k] = 0;
		/* 直接用 filp_open 探测：存在就能打开 */
		{
			struct file *f = filp_open(cand, O_RDONLY, 0);

			if (!IS_ERR(f)) {
				filp_close(f, NULL);
				strscpy(out, cand, outlen);
				return 0;
			}
		}
	}
	/* 都不存在：返回第一个候选，调用方按"空文件"处理 */
	k = 0;
	k += scnprintf(cand + k, sizeof(cand) - k, "%s", pearl_fs_roots[0]);
	{
		const char *p = rest;

		while (*p != 0 && k + 1 < sizeof(cand)) {
			cand[k++] = (*p == '\\') ? '/' : *p;
			p++;
		}
	}
	cand[k] = 0;
	strscpy(out, cand, outlen);
	return -1;
}

/* 读整个文件（最多 maxlen 字节）；返回实际读到的字节数，失败返回 -1 */
static int pearl_fs_read_file(const char *lpath, unsigned char *out,
	unsigned int maxlen)
{
	struct file *f;
	loff_t pos = 0;
	int ret;

	f = filp_open(lpath, O_RDONLY, 0);
	if (IS_ERR(f))
		return -1;
	ret = kernel_read(f, out, maxlen, &pos);
	filp_close(f, NULL);
	return (ret < 0) ? -1 : ret;
}

static void pearl_fs_job_fn(struct work_struct *work)
{
	struct pearl_fs_job *job = pearl_fs_job;
	unsigned char *req = job->data;
	unsigned int req_len = job->len;
	unsigned char reply[PEARL_FS_MAX_MSG];
	const unsigned char *blk[PEARL_FS_MAX_BLK];
	unsigned int blk_len[PEARL_FS_MAX_BLK];
	unsigned int op, req_blk, i, off, pos, nblk = 0;
	unsigned int status = 0, out = 0, handle = 0, mode = 0;
	unsigned int blk3val = 0;
	static unsigned char databuf[4096];
	char name[96];
	int idx, hidx, cnt, j;

	if (req_len < 24) {
		pr_err("PEARL-FS: request too short (%u)\n", req_len);
		goto out;
	}
	op = *(unsigned int *)(req + 16);
	req_blk = *(unsigned int *)(req + 20);
	off = 24;
	for (i = 0; i < req_blk && i < PEARL_FS_MAX_BLK; i++) {
		unsigned int l;

		if (off + sizeof(unsigned int) > req_len)
			break;
		l = *(unsigned int *)(req + off);
		off += sizeof(unsigned int);
		if (l > req_len - off)
			break;
		blk[i] = req + off;
		blk_len[i] = l;
		off += (l + 3) & ~3U;
	}
	name[0] = 0;
	if (i >= 2 && op == PEARL_FS_OP_OPEN && blk_len[1] >= 4)
		mode = *(unsigned int *)blk[1];
	if ((op == PEARL_FS_OP_OPEN || op == PEARL_FS_OP_CMPT_READ) && i >= 1)
		pearl_fs_wcs2cs(blk[0], blk_len[0], name, sizeof(name));
	else if (i >= 1 && blk_len[0] >= 4)
		handle = *(unsigned int *)blk[0];

	mutex_lock(&pearl_fs_lock);
	hidx = -1;
	if (handle >= 1 && handle <= PEARL_FS_MAX_HANDLE &&
	    pearl_fs_handles[handle - 1].used)
		hidx = handle - 1;

	memcpy(reply, req, sizeof(struct ccci_header));
	reply[8] = CCCI_FS_TX;	/* channel 低字节；第 9 字节本来就是 0 */
	*(unsigned int *)(reply + 16) = op | 0xFFFF0000U;
	pos = 24;
	switch (op) {
	case PEARL_FS_OP_OPEN:
		idx = pearl_fs_file_find(name);
		if (idx < 0)
			idx = pearl_fs_file_new(name);
		if (idx < 0) {
			status = 1;
			handle = 0;
		} else {
			handle = pearl_fs_handle_alloc(idx);
			if (handle == 0)
				status = 1;
		}
		if (handle != 0) {
			char lpath[256];
			struct file *f;
			struct pearl_fs_handle *h =
				&pearl_fs_handles[handle - 1];

			pearl_fs_map_path(name, lpath, sizeof(lpath));
			f = filp_open(lpath, O_RDWR | O_CREAT, 0660);
			if (!IS_ERR(f)) {
				h->fp = f;
				h->pos = 0;
				pearl_fs_files[h->file].size =
					(unsigned int)i_size_read(file_inode(f));
			} else {
				pr_err("PEARL-FS: open %s -> %s fail %ld\n",
					name, lpath, PTR_ERR(f));
			}
		}
		pos = pearl_fs_put_block(reply, pos, &handle, 4);
		nblk = 1;
		break;
	case PEARL_FS_OP_SEEK:
	{
		int whence = 0;
		unsigned int seek_off = 0;
		struct pearl_fs_file *f;

		if (hidx < 0) {
			status = 1;
			break;
		}
		f = &pearl_fs_files[pearl_fs_handles[hidx].file];
		if (i >= 2 && blk_len[1] >= 4)
			seek_off = *(unsigned int *)blk[1];
		if (i >= 3 && blk_len[2] >= 4)
			whence = (int)*(unsigned int *)blk[2];
		if (pearl_fs_handles[hidx].fp != NULL) {
			loff_t np = vfs_llseek(pearl_fs_handles[hidx].fp,
				seek_off, whence);

			if (np < 0) {
				status = 1;
				break;
			}
			pearl_fs_handles[hidx].pos = (unsigned int)np;
		} else if (whence == 1) {
			pearl_fs_handles[hidx].pos += seek_off;
		} else if (whence == 2) {
			pearl_fs_handles[hidx].pos = f->size + seek_off;
		} else {
			pearl_fs_handles[hidx].pos = seek_off;
		}
		out = pearl_fs_handles[hidx].pos;
		pos = pearl_fs_put_block(reply, pos, &out, 4);
		nblk = 1;
		break;
	}
	case PEARL_FS_OP_WRITE:
	{
		unsigned int wlen = (i >= 2) ? blk_len[1] : 0;
		unsigned int woff;
		struct pearl_fs_file *f;

		if (hidx < 0) {
			status = 1;
			pos = pearl_fs_put_block(reply, pos, &status, 4);
			nblk = 1;
			break;
		}
		f = &pearl_fs_files[pearl_fs_handles[hidx].file];
		/*
		 * PEARL-FS-WOFF: 用"文件当前位置"而不是 block3。
		 * 基带的动作是 Seek(SEEK_END) 之后再 Write（POSIX 语义），
		 * 实测把 block3 当偏移会写到 0 —— 表现是 nv_boot_trace
		 * 永远不增长（我们写成功了但文件大小不变）。block3 只记日志。
		 */
		woff = pearl_fs_handles[hidx].pos;
		if (i >= 3 && blk_len[2] >= 4)
			blk3val = *(unsigned int *)blk[2];
		if (pearl_fs_handles[hidx].fp != NULL &&
		    strstr(f->name, "NVRAM") == NULL) {
			loff_t wpos = woff;
			ssize_t wr = kernel_write(pearl_fs_handles[hidx].fp,
				blk[1], wlen, &wpos);

			if (wr < 0)
				status = 1;
			else
				out = (unsigned int)wr;
			pearl_fs_handles[hidx].pos = woff + wlen;
			if (woff + wlen > f->size)
				f->size = woff + wlen;
		} else if (pearl_fs_file_reserve(f, woff + wlen) == 0) {
			memcpy(f->data + woff, blk[1], wlen);
			if (woff + wlen > f->size)
				f->size = woff + wlen;
			pearl_fs_handles[hidx].pos = woff + wlen;
			out = wlen;
		} else {
			status = 1;
		}
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		nblk = 1;
		break;
	}
	case PEARL_FS_OP_READ:
	{
		unsigned int rlen = 0, roff;
		struct pearl_fs_file *f;
		unsigned char tmp[2048];

		if (hidx < 0) {
			status = 1;
		} else {
			f = &pearl_fs_files[pearl_fs_handles[hidx].file];
			roff = pearl_fs_handles[hidx].pos;
			if (i >= 2 && blk_len[1] >= 4)
				rlen = *(unsigned int *)blk[1];
			if (rlen > sizeof(tmp))
				rlen = sizeof(tmp);
			memset(tmp, 0, sizeof(tmp));
			if (pearl_fs_handles[hidx].fp != NULL) {
				loff_t rpos = roff;
				ssize_t rd = kernel_read(
					pearl_fs_handles[hidx].fp, tmp, rlen,
					&rpos);

				if (rd < 0) {
					status = 1;
					rd = 0;
				}
				pearl_fs_handles[hidx].pos = roff + rd;
				out = rd;
			} else {
				if (roff >= f->size)
					rlen = 0;
				else if (roff + rlen > f->size)
					rlen = f->size - roff;
				if (rlen > 0)
					memcpy(tmp, f->data + roff, rlen);
				pearl_fs_handles[hidx].pos = roff + rlen;
				out = rlen;
			}
		}
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		pos = pearl_fs_put_block(reply, pos, tmp, out);
		nblk = 2;
		break;
	}
	case PEARL_FS_OP_FILE_SIZE:
		if (hidx < 0) {
			status = 1;
			out = 0;
		} else if (pearl_fs_handles[hidx].fp != NULL) {
			out = (unsigned int)i_size_read(
				file_inode(pearl_fs_handles[hidx].fp));
			pearl_fs_files[pearl_fs_handles[hidx].file].size = out;
		} else {
			out = pearl_fs_files[pearl_fs_handles[hidx].file].size;
		}
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		pos = pearl_fs_put_block(reply, pos, &out, 4);
		nblk = 2;
		break;
	case PEARL_FS_OP_CLOSE:
		if (hidx >= 0) {
			if (pearl_fs_handles[hidx].fp != NULL)
				filp_close(pearl_fs_handles[hidx].fp, NULL);
			memset(&pearl_fs_handles[hidx], 0,
				sizeof(pearl_fs_handles[hidx]));
		} else {
			status = 1;
		}
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		nblk = 1;
		break;
	case PEARL_FS_OP_CLOSE_ALL:
		for (idx = 0; idx < PEARL_FS_MAX_HANDLE; idx++)
			if (pearl_fs_handles[idx].fp != NULL)
				filp_close(pearl_fs_handles[idx].fp, NULL);
		memset(pearl_fs_handles, 0, sizeof(pearl_fs_handles));
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		nblk = 1;
		break;
	case PEARL_FS_OP_CMPT_READ:
	{
		/*
		 * 实测（yuechu strace 4 个样本一致）：
		 *   请求 blk1 = {len: path(UTF-16LE)}
		 *   请求 blk2 = 40 字节描述符:
		 *     w0=步骤位图  w1=状态  w2/w3=flags  w4=缓冲地址
		 *     w5..w7=?     w8=要读取的长度  w9=?
		 *   回复 nblk=4: {8: [w0][状态]} {4: w4} {4: 实际长度} {数据}
		 * 处理链 = Open + GetFileSize + Seek + Read + Close（读整个文件）。
		 */
		unsigned int steps = 0, astat = 0, bufaddr = 0, want = 0;
		char lpath[256];
		int got = -1;

		if (i >= 2 && blk_len[1] >= 8) {
			steps = *(unsigned int *)blk[1];
			astat = 0;
		}
		if (i >= 2 && blk_len[1] >= 40) {
			bufaddr = *(unsigned int *)(blk[1] + 16);
			want = *(unsigned int *)(blk[1] + 32);
		} else if (i >= 3 && blk_len[2] >= 4) {
			want = *(unsigned int *)blk[2];
		}
		if (want == 0 || want > sizeof(databuf))
			want = sizeof(databuf);
		memset(databuf, 0, sizeof(databuf));
		pearl_fs_map_path(name, lpath, sizeof(lpath));
		got = pearl_fs_read_file(lpath, databuf, want);
		if (got < 0) {
			pr_err("PEARL-FS: CMPTREAD miss %s -> %s\n",
				name, lpath);
			got = 0;
		}
		out = got;
		/* 步骤位图：0x1d = open|seek|read|close 都做过（照 Android 观测值） */
		if (steps == 0)
			steps = 0x1d;
		{
			unsigned int hdr[2];

			hdr[0] = steps;
			hdr[1] = astat;
			pos = pearl_fs_put_block(reply, pos, hdr, sizeof(hdr));
		}
		pos = pearl_fs_put_block(reply, pos, &bufaddr, 4);
		pos = pearl_fs_put_block(reply, pos, &out, 4);
		pos = pearl_fs_put_block(reply, pos, databuf, out);
		nblk = 4;
		break;
	}
	default:
		/* 未实现的 op（FindFirst/GetAttributes/Restore/...）：回 MTK 的通用错误码，
		 * 而不是 1 —— 基带把回复里的值当错误码用（实测它把我们的 1 显示成 -1001）。
		 */
		status = 0xFFFFFC17;	/* -1001，与基带 trace 里看到的一致 */
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		nblk = 1;
		break;
	}
	*(unsigned int *)(reply + 20) = nblk;
	*(unsigned int *)(reply + 4) = pos;
	mutex_unlock(&pearl_fs_lock);

	cnt = atomic_inc_return(&pearl_fs_msg_cnt);
	if (cnt <= 96)
		pr_err("PEARL-FS: #%d op=0x%04x seq=%u req=%u nblk=%u h=%u mode=0x%x st=%u out=%u b3=0x%x name=%s -> rep=%u\n",
			cnt, op, ((struct ccci_header *)req)->seq_num, req_len,
			req_blk, handle, mode, status, out, blk3val, name, pos);
	if (cnt <= 40)
		print_hex_dump(KERN_ERR, "PEARL-FS-REQ: ", DUMP_PREFIX_OFFSET,
			16, 1, req, req_len < 128 ? req_len : 128, false);

	pearl_fs_send(job->md_id, reply, pos);
out:
	j = 0;
	(void)j;
	kfree(job);
	pearl_fs_job = NULL;
}

int pearl_fs_handle_rx(unsigned char md_id, const unsigned char *msg,
	unsigned int len)
{
	struct pearl_fs_job *job;

	if (pearl_fs_mode == 0 || len < 24 || len > PEARL_FS_MAX_MSG)
		return 0;
	/* 可能在中断上下文被调用：只做拷贝 + 丢给工作队列；上一单没跑完就丢 */
	if (pearl_fs_job != NULL)
		return 0;
	job = kmalloc(sizeof(*job) + len, GFP_ATOMIC);
	if (job == NULL)
		return -ENOMEM;
	INIT_WORK(&job->work, pearl_fs_job_fn);
	job->md_id = md_id;
	job->len = len;
	memcpy(job->data, msg, len);
	pearl_fs_job = job;
	schedule_work(&job->work);
	return 1;
}
EXPORT_SYMBOL(pearl_fs_handle_rx);

static int __init pearl_fs_init(void)
{
	pearl_fs_proc_init();
	return 0;
}
late_initcall(pearl_fs_init);


static void pearl_amms_dump(int md_id, const unsigned char *p,
	unsigned int len, const char *tag)
{
	unsigned int k;

	CCCI_ERROR_LOG(md_id, RPC, "PEARL-AMMS %s len=%u\n", tag, len);
	if (!pearl_amms_dump_req)
		return;
	for (k = 0; k + 16 <= len; k += 16)
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS %s %03u: %08x %08x %08x %08x\n", tag, k,
			*(u32 *)(p + k), *(u32 *)(p + k + 4),
			*(u32 *)(p + k + 8), *(u32 *)(p + k + 12));
	if (k < len) {
		u32 w[4] = {0, 0, 0, 0};
		unsigned int i;

		for (i = 0; i < 4 && (k + i * 4) < len; i++)
			memcpy(&w[i], p + k + i * 4,
				min_t(unsigned int, 4, len - k - i * 4));
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS %s %03u: %08x %08x %08x %08x\n",
			tag, k, w[0], w[1], w[2], w[3]);
	}
}

/* COPY：按 {src,dst,len} 把 md1drdi 数据搬进 64KiB DRDI smem */
static int pearl_amms_copy_sets(int md_id, int slot, struct pearl_amms_req *req)
{
	struct ccci_smem_region *smem;
	struct pearl_amms_set_copy *cs;
	void __iomem *dst_base;
	unsigned int i, num = pearl_amms_set_total[slot];
	int ret = 0;

	if (!num || num > PEARL_AMMS_MAX_SET) {
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS: no valid set_total_num (%u), use req value %u\n",
			num, req->set_total_num);
		num = req->set_total_num;
	}
	smem = ccci_md_get_smem_by_user_id(md_id, SMEM_USER_MD_DRDI);
	if (!smem || !pearl_drdi_data) {
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS: smem=%p drdi=%p, cannot copy\n",
			smem, pearl_drdi_data);
		return -ENODEV;
	}
	/* PEARL-AMMS: DRDI smem 必须非 cache —— 见 scripts/patch-amms-drdi-wc.py 的说明。
	 * 厂商把 SMEM_USER_MD_DRDI 放在 md1_6297_noncacheable_fat[]，userspace
	 * ccci_rpcd 也用 pgprot_noncached 映射它。memremap(MEMREMAP_WB) 会拿到
	 * cacheable 线性映射，MD 走总线读 DRAM 会读到旧数据（并造成同一物理页
	 * 的 cacheable/non-cacheable 别名）。这里统一用 ioremap_wc，与同文件
	 * NVRAM cache 路径一致。
	 */
	dst_base = ioremap_wc(smem->base_ap_view_phy, smem->size);
	if (!dst_base) {
		CCCI_ERROR_LOG(md_id, RPC, "PEARL-AMMS: ioremap_wc smem fail\n");
		return -ENOMEM;
	}
	/* COPY 表从 req+0x08 开始，stride 12：{src, dst, len}
	 * （ccci_rpcd 反汇编：x9=req+0x10 指向 len，src=[x9-8], dst=[x9-4]）
	 */
	cs = (struct pearl_amms_set_copy *)req->tbl;
	for (i = 0; i < num && i < PEARL_AMMS_MAX_SET; i++) {
		u32 src = cs[i].src, dst = cs[i].dst, len = cs[i].len;

		if (!len)
			continue;
		if (len > PEARL_AMMS_MAX_COPY_LEN || src + len > pearl_drdi_len ||
		    dst + len > smem->size) {
			CCCI_ERROR_LOG(md_id, RPC,
				"PEARL-AMMS copy set(%u) bad: src=0x%x dst=0x%x len=0x%x (img 0x%x smem 0x%x)\n",
				i, src, dst, len, pearl_drdi_len, smem->size);
			ret = -ERANGE;
			continue;
		}
		memcpy_toio(dst_base + dst, pearl_drdi_data + src, len);
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS copy set(%u) from 0x%x to smem+0x%x len=0x%x\n",
			i, src, dst, len);
	}
	/* 写序：确保拷贝在应答发出前落到 DRAM（MD 与 AP 不缓存一致） */
	wmb();
	iounmap(dst_base);
	return ret;
}

/* 处理一条 AMMS DRDI 请求：写 opkt[]，返回参数个数（应答固定 2 个参数） */
/* ===== PEARL-MDLOG-AMMS-BEGIN ===== */
/* PEARL-MDLOG: 任务要求的「首次 AMMS 触发」——AMMS 说明基带已经起来并在通信，
 * 此时顺手把日志使能消息发一次（只发一次；同样受 pearl_mdlog_auto 开关约束）。 */
extern int pearl_mdlog_send_armed(unsigned int msg, unsigned int resv,
	int blocking, const char *why);
extern void pearl_mdlog_kick(void);

static void pearl_mdlog_amms_kick(void)
{
	/* PEARL-MDLOG-EXC: 这里不能发。
	 *
	 * AMMS 请求正好出现在基带启动握手的 HS1/HS2 窗口里（约 7.7-8.1s），
	 * 而厂商 port_proxy.c 在这个窗口明确禁止一切非 FS/RPC 端口流量；实验 M
	 * 证明此时发 0x0C 会让基带固定在 HS1+5.43s 断言（ccismcore_ccci.c:1326）。
	 * 拉日志改由进入 EXCEPTION 时的 pearl_mdlog_kick() 负责。
	 */
}
/* ===== PEARL-MDLOG-AMMS-END ===== */








static int pearl_amms_handle(struct port_t *port, struct rpc_buffer *rpc_buf,
	struct rpc_pkt *pkt, int pkt_num, struct rpc_pkt *opkt, u32 *tmp_data)
{
	int md_id = port->md_id;
	int slot = md_id & 1;
	struct pearl_amms_req *req;
	struct pearl_amms_rsp *rsp;
	struct pearl_amms_set_init *is;
	u32 *ret_code = &tmp_data[0];
	int i, copy_ret = 0;
	bool fail = false;

	pearl_mdlog_amms_kick();

	if (pkt_num < 1 || pkt[0].len < PEARL_AMMS_REQ_SIZE) {
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS bad request pkt_num=%d len=%u (need %u)\n",
			pkt_num, pkt_num > 0 ? pkt[0].len : 0,
			PEARL_AMMS_REQ_SIZE);
		tmp_data[0] = FS_PARAM_ERROR;
		opkt[0].len = sizeof(u32);
		opkt[0].buf = (void *)&tmp_data[0];
		return 1;
	}
	req = (struct pearl_amms_req *)pkt[0].buf;
	rsp = (struct pearl_amms_rsp *)&tmp_data[1];
	memset(rsp, 0, sizeof(*rsp));
	rsp->seq_id = req->seq_id;
	*ret_code = 0;
	pearl_amms_req_cnt[slot]++;
	/* 与 ccci_rpcd 一致：清掉 data[0] 的"还有分片"位 */
	rpc_buf->header.data[0] &= ~0x80000000U;

	pearl_amms_dump(md_id, (const unsigned char *)req, pkt[0].len,
		(req->cmd == PEARL_AMMS_CMD_INIT) ? "init" : "copy");
	CCCI_ERROR_LOG(md_id, RPC,
		"PEARL-AMMS cmd=%u seq=%u ver=%u set_total=%u cnt=%u\n",
		req->cmd, req->seq_id, req->ver, req->set_total_num,
		pearl_amms_req_cnt[slot]);

	switch (req->cmd) {
	case PEARL_AMMS_CMD_INIT:
		/*
		 * PEARL: CCCI 内建后 port_rpc_init 在 ~2.7s 就返回了，那一刻
		 * /dev/disk/by-partlabel 尚未建立（udev 还没跑），
		 * pearl_drdi_load_image() 打不开 md1img 分区，pearl_drdi_data
		 * 恒为 NULL，AMMS init 只能回 0xFFFFFFFF。这里改成与 NVRAM
		 * 缓存同样的懒加载：真正要用的时候再读一次。
		 */
		if (!pearl_drdi_data)
			pearl_drdi_load_image();
		/* MODEM 马上要读 RF/NVRAM 数据了：先把 NVRAM cache 区灌好 */
		pearl_nvram_fill_cache(md_id, 1);
		pearl_amms_set_total[slot] = req->set_total_num;
		is = (struct pearl_amms_set_init *)req->tbl;
		for (i = 0; i < PEARL_AMMS_MAX_SET; i++)
			if (is[i].len)
				CCCI_ERROR_LOG(md_id, RPC,
					"PEARL-AMMS set[%d] off=0x%x len=0x%x\n",
					i, is[i].off, is[i].len);
		if (req->ver != PEARL_AMMS_INIT_VER) {
			CCCI_ERROR_LOG(md_id, RPC,
				"PEARL-AMMS init version(%u) error\n", req->ver);
			fail = true;
		} else if (req->set_total_num > PEARL_AMMS_MAX_SET) {
			CCCI_ERROR_LOG(md_id, RPC,
				"PEARL-AMMS init set_total_num(%u) error\n",
				req->set_total_num);
			fail = true;
		} else if (!pearl_drdi_data) {
			CCCI_ERROR_LOG(md_id, RPC,
				"PEARL-AMMS init: no drdi image\n");
			fail = true;
		} else {
			for (i = 0; i < req->set_total_num; i++) {
				if (is[i].off + is[i].len > pearl_drdi_len) {
					CCCI_ERROR_LOG(md_id, RPC,
						"PEARL-AMMS init set[%d] out of range off=0x%x len=0x%x (img 0x%x)\n",
						i, is[i].off, is[i].len,
						pearl_drdi_len);
					fail = true;
					break;
				}
			}
		}
		rsp->ver = PEARL_AMMS_INIT_VER;
		if (fail) {
			rsp->stats = 0xFF;
			rsp->drdiinfostat = 0xFF;
			*ret_code = 0xFFFFFFFF;
		} else {
			rsp->stats = 0;
			rsp->copystat =
				(pearl_amms_copy_done[slot] == 1) ? 0xFF : 0;
			rsp->drdiinfostat = 0;
		}
		break;

	case PEARL_AMMS_CMD_COPY:
		pearl_amms_copy_done[slot] = (req->ver == PEARL_AMMS_COPY_VER);
		rsp->ver = (req->ver == PEARL_AMMS_COPY_VER) ? 0 : 0xFF;
		if (pearl_amms_do_copy)
			copy_ret = pearl_amms_copy_sets(md_id, slot, req);
		/* 完全照 rpcd 行为：copy_done 不为 -1 时 stats=0，ret=0 */
		rsp->stats = 0;
		*ret_code = 0;
		if (copy_ret)
			CCCI_ERROR_LOG(md_id, RPC,
				"PEARL-AMMS copy error %d (still reply success like rpcd)\n",
				copy_ret);
		break;

	default:
		CCCI_ERROR_LOG(md_id, RPC, "PEARL-AMMS unknown cmd %u\n",
			req->cmd);
		rsp->stats = 0xFF;
		*ret_code = 0xFFFFFFFF;
		break;
	}

	if (pearl_amms_fail_ok) {
		*ret_code = 0xFFFFFFFF;
		rsp->stats = 0xFF;
	}
	CCCI_ERROR_LOG(md_id, RPC,
		"PEARL-AMMS reply ret=0x%x stats=0x%x seq=%u ver=%u copystat=0x%x drdiinfo=0x%x\n",
		*ret_code, rsp->stats, rsp->seq_id, rsp->ver, rsp->copystat,
		rsp->drdiinfostat);

	opkt[0].len = sizeof(u32);
	opkt[0].buf = (void *)ret_code;
	opkt[1].len = sizeof(struct pearl_amms_rsp);
	opkt[1].buf = (void *)rsp;
	return 2;
}

static void ccci_rpc_work_helper(struct port_t *port, struct rpc_pkt *pkt,
	struct rpc_buffer *p_rpc_buf, unsigned int tmp_data[])
{
	/*
	 * tmp_data[] is used to make sure memory address is valid
	 * after this function return, be careful with the size!
	 */
	int pkt_num = p_rpc_buf->para_num;
	int md_id = port->md_id;
	int md_val = -1;

	CCCI_DEBUG_LOG(md_id, RPC, "%s++ %d\n", __func__,
		p_rpc_buf->para_num);
	tmp_data[0] = 0;
	switch (p_rpc_buf->op_id) {
	/* call EINT API to get TDD EINT configuration for modem EINT initial */
	case IPC_RPC_GET_TDD_EINT_NUM_OP:
	case IPC_RPC_GET_GPIO_NUM_OP:
	case IPC_RPC_GET_ADC_NUM_OP:
		{
			int get_num = 0;
			unsigned char *name = NULL;
			unsigned int length = 0;

			if (pkt_num < 2 || pkt_num > RPC_MAX_ARG_NUM) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d!\n",
				p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err1;
			}
			length = pkt[0].len;
			if (length < 1) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d, name_len=%d!\n",
				p_rpc_buf->op_id, pkt_num, length);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err1;
			}

			name = kmalloc(length, GFP_KERNEL);
			if (name == NULL) {
				CCCI_ERROR_LOG(md_id, RPC,
				"Fail alloc Mem for [0x%X]!\n",
				p_rpc_buf->op_id);
				tmp_data[0] = FS_ERROR_RESERVED;
				goto err1;
			} else {
				memcpy(name, (unsigned char *)(pkt[0].buf),
				length);

				if (p_rpc_buf->op_id ==
					IPC_RPC_GET_TDD_EINT_NUM_OP) {
					get_num = get_td_eint_info(name,
								length);
					if (get_num < 0)
						get_num = FS_FUNC_FAIL;
				} else if (p_rpc_buf->op_id ==
						IPC_RPC_GET_GPIO_NUM_OP) {
					get_num = get_md_gpio_info(name,
								length,
								&md_val);
					if (get_num < 0)
						get_num = FS_FUNC_FAIL;
					else
						get_num = md_val;
				} else if (p_rpc_buf->op_id ==
						IPC_RPC_GET_ADC_NUM_OP) {
					get_num = get_md_adc_info(name,
								length);
					if (get_num < 0)
						get_num = FS_FUNC_FAIL;
				}

				CCCI_NORMAL_LOG(md_id, RPC,
					"[0x%08X]: name:%s, len=%d, get_num:%d\n",
					p_rpc_buf->op_id, name,
					length, get_num);
				pkt_num = 0;

				/* NOTE: tmp_data[1] not [0] */
				tmp_data[1] = (unsigned int)get_num;
				/* get_num may be invalid after
				 * exit this function
				 */
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)(&tmp_data[1]);
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)(&tmp_data[1]);
				kfree(name);
			}
			break;

 err1:
			pkt_num = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			break;
		}

	case IPC_RPC_GET_EMI_CLK_TYPE_OP:
		{
			int dram_type = 0;
			int dram_clk = 0;

			if (pkt_num != 0) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d!\n",
				p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err2;
			}

			if (get_dram_type_clk(&dram_clk, &dram_type)) {
				tmp_data[0] = FS_FUNC_FAIL;
				goto err2;
			} else {
				tmp_data[0] = 0;
				CCCI_NORMAL_LOG(md_id, RPC,
				"[0x%08X]: dram_clk: %d, dram_type:%d\n",
				p_rpc_buf->op_id, dram_clk, dram_type);
			}

			tmp_data[1] = (unsigned int)dram_type;
			tmp_data[2] = (unsigned int)dram_clk;

			pkt_num = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)(&tmp_data[0]);
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)(&tmp_data[1]);
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)(&tmp_data[2]);
			break;

 err2:
			pkt_num = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			break;
		}

	case IPC_RPC_GET_EINT_ATTR_OP:
		{
			char *eint_name = NULL;
			unsigned int name_len = 0;
			unsigned int type = 0;
			char *res = NULL;
			unsigned int res_len = 0;
			int ret = 0;

			if (pkt_num < 3 || pkt_num > RPC_MAX_ARG_NUM) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d!\n",
				p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err3;
			}
			name_len = pkt[0].len;
			if (name_len < 1) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d, name_len=%d!\n",
				p_rpc_buf->op_id, pkt_num, name_len);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err3;
			}

			eint_name = kmalloc(name_len, GFP_KERNEL);
			if (eint_name == NULL) {
				CCCI_ERROR_LOG(md_id, RPC,
				"Fail alloc Mem for [0x%X]!\n",
				p_rpc_buf->op_id);
				tmp_data[0] = FS_ERROR_RESERVED;
				goto err3;
			} else {
				memcpy(eint_name, (unsigned char *)(pkt[0].buf),
				name_len);
			}

			type = *(unsigned int *)(pkt[2].buf);
			res = (unsigned char *)&(p_rpc_buf->para_num) +
					4 * sizeof(unsigned int);
			ret = get_eint_attr(md_id, eint_name, name_len, type,
					res, &res_len);
			if (ret == 0) {
				tmp_data[0] = ret;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = res_len;
				pkt[pkt_num++].buf = (void *)res;
				CCCI_DEBUG_LOG(md_id, RPC,
					"[0x%08X] OK: name:%s, len:%d, type:%d, res:%d, res_len:%d\n",
					p_rpc_buf->op_id, eint_name, name_len,
					type, *res, res_len);
				kfree(eint_name);
			} else {
				tmp_data[0] = ret;
				CCCI_DEBUG_LOG(md_id, RPC,
					"[0x%08X] fail: name:%s, len:%d, type:%d, ret:%d\n",
					p_rpc_buf->op_id, eint_name, name_len,
					type, ret);
				kfree(eint_name);
				goto err3;
			}
			break;

 err3:
			pkt_num = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			break;
		}
#ifdef FEATURE_RF_CLK_BUF
	case IPC_RPC_GET_RF_CLK_BUF_OP:
		{
			u16 count = 0;
			struct ccci_rpc_clkbuf_result *clkbuf;
			CLK_BUF_SWCTRL_STATUS_T swctrl_status[CLKBUF_MAX_COUNT];
			struct ccci_rpc_clkbuf_input *clkinput;
			u32 AfcDac;
			int ret = 0;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}
			clkinput = (struct ccci_rpc_clkbuf_input *)pkt[0].buf;
			AfcDac = clkinput->AfcCwData;
			count = clkinput->CLKBuf_Num;
			pkt_num = 0;
			tmp_data[0] = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len =
				sizeof(struct ccci_rpc_clkbuf_result);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
			clkbuf = (struct ccci_rpc_clkbuf_result *)&tmp_data[1];
			if (count != CLKBUF_MAX_COUNT) {
				CCCI_ERROR_LOG(md_id, RPC,
				"IPC_RPC_GET_RF_CLK_BUF, wrong count %d/%d\n",
				count, CLKBUF_MAX_COUNT);
				clkbuf->CLKBuf_Count = 0xFF;
				memset(&clkbuf->CLKBuf_Status, 0,
					sizeof(clkbuf->CLKBuf_Status));
			} else if (is_clk_buf_from_pmic()) {
				clkbuf->CLKBuf_Count = CLKBUF_MAX_COUNT;
				memset(&clkbuf->CLKBuf_Status, 0,
					sizeof(clkbuf->CLKBuf_Status));
				memset(&clkbuf->CLKBuf_SWCtrl_Status, 0,
					sizeof(clkbuf->CLKBuf_SWCtrl_Status));
				memset(&clkbuf->ClkBuf_Driving, 0,
					sizeof(clkbuf->ClkBuf_Driving));
			} else {
				unsigned int vals_drv[CLKBUF_MAX_COUNT] = {
					2, 2, 2, 2};
				u32 vals[CLKBUF_MAX_COUNT] = {0, 0, 0, 0};
				struct device_node *node;

				node = of_find_compatible_node(NULL, NULL,
						"mediatek,rf_clock_buffer");
				if (node) {
					ret = of_property_read_u32_array(node,
						"mediatek,clkbuf-config", vals,
						CLKBUF_MAX_COUNT);

					if (ret)
						CCCI_ERROR_LOG(md_id, RPC,
							"%s get property fail\n",
							__func__);

				} else {
					CCCI_ERROR_LOG(md_id, RPC,
					"%s can't find compatible node\n",
					__func__);
				}
				clkbuf->CLKBuf_Count = CLKBUF_MAX_COUNT;
				clkbuf->CLKBuf_Status[0] = vals[0];
				clkbuf->CLKBuf_Status[1] = vals[1];
				clkbuf->CLKBuf_Status[2] = vals[2];
				clkbuf->CLKBuf_Status[3] = vals[3];
				clk_buf_get_swctrl_status(swctrl_status);
				clk_buf_get_rf_drv_curr(vals_drv);
				clk_buf_save_afc_val(AfcDac);
				clkbuf->CLKBuf_SWCtrl_Status[0] =
					swctrl_status[0];
				clkbuf->CLKBuf_SWCtrl_Status[1] =
					swctrl_status[1];
				clkbuf->CLKBuf_SWCtrl_Status[2] =
					swctrl_status[2];
				clkbuf->CLKBuf_SWCtrl_Status[3] =
					swctrl_status[3];
				clkbuf->ClkBuf_Driving[0] = vals_drv[0];
				clkbuf->ClkBuf_Driving[1] = vals_drv[1];
				clkbuf->ClkBuf_Driving[2] = vals_drv[2];
				clkbuf->ClkBuf_Driving[3] = vals_drv[3];
				CCCI_NORMAL_LOG(md_id, RPC,
					"RF_CLK_BUF*_DRIVING_CURR %d, %d, %d, %d, AfcDac: %d\n",
					vals_drv[0], vals_drv[1], vals_drv[2],
					vals_drv[3], AfcDac);
			}
			CCCI_DEBUG_LOG(md_id, RPC,
				"IPC_RPC_GET_RF_CLK_BUF count=%x\n",
				clkbuf->CLKBuf_Count);
			break;
		}
#endif
	case IPC_RPC_GET_GPIO_VAL_OP:
	case IPC_RPC_GET_ADC_VAL_OP:
		{
			unsigned int num = 0;
			int val = 0;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err4;
			}

			num = *(unsigned int *)(pkt[0].buf);
			if (p_rpc_buf->op_id == IPC_RPC_GET_GPIO_VAL_OP)
				val = get_md_gpio_val(num);
			else if (p_rpc_buf->op_id == IPC_RPC_GET_ADC_VAL_OP)
				val = get_md_adc_val(num);
			tmp_data[0] = val;
			CCCI_DEBUG_LOG(md_id, RPC, "[0x%X]: num=%d, val=%d!\n",
				p_rpc_buf->op_id, num, val);

 err4:
			pkt_num = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			break;
		}

	case IPC_RPC_GET_GPIO_ADC_OP:
		{
			struct ccci_rpc_gpio_adc_intput *input;
			struct ccci_rpc_gpio_adc_output *output;
			struct ccci_rpc_gpio_adc_intput_v2 *input_v2;
			struct ccci_rpc_gpio_adc_output_v2 *output_v2;
			unsigned int pkt_size;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}
			pkt_size = pkt[0].len;
			if (pkt_size ==
				sizeof(struct ccci_rpc_gpio_adc_intput)) {
				input =
				(struct ccci_rpc_gpio_adc_intput *)(pkt[0].buf);
				pkt_num = 0;
				tmp_data[0] = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len =
				sizeof(struct ccci_rpc_gpio_adc_output);
				pkt[pkt_num++].buf = (void *)&tmp_data[1];
				output =
				(struct ccci_rpc_gpio_adc_output *)&tmp_data[1];
				/* 0xF for failure */
				memset(output, 0xF,
				sizeof(struct ccci_rpc_gpio_adc_output));
				CCCI_BOOTUP_LOG(md_id, RPC,
					"IPC_RPC_GET_GPIO_ADC_OP request=%x\n",
					input->reqMask);
				ccci_rpc_get_gpio_adc(input, output);
			} else if (pkt_size ==
				sizeof(struct ccci_rpc_gpio_adc_intput_v2)) {
				input_v2 =
				(struct ccci_rpc_gpio_adc_intput_v2 *)
				(pkt[0].buf);
				pkt_num = 0;
				tmp_data[0] = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len =
				sizeof(struct ccci_rpc_gpio_adc_output_v2);
				pkt[pkt_num++].buf = (void *)&tmp_data[1];
				output_v2 =
				(struct ccci_rpc_gpio_adc_output_v2 *)
				&tmp_data[1];
				/* 0xF for failure */
				memset(output_v2, 0xF,
				sizeof(struct ccci_rpc_gpio_adc_output_v2));
				CCCI_BOOTUP_LOG(md_id, RPC,
					"IPC_RPC_GET_GPIO_ADC_OP request=%x\n",
					input_v2->reqMask);
				ccci_rpc_get_gpio_adc_v2(input_v2, output_v2);
			} else {
				CCCI_ERROR_LOG(md_id, RPC,
					"can't recognize pkt size%d!\n",
					pkt_size);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
			}
			break;
		}

#ifdef FEATURE_INFORM_NFC_VSIM_CHANGE
	case IPC_RPC_USIM2NFC_OP:
		{
			struct ccci_rpc_usim2nfs *input, *output;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}
			input = (struct ccci_rpc_usim2nfs *)(pkt[0].buf);
			pkt_num = 0;
			tmp_data[0] = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(struct ccci_rpc_usim2nfs);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
			output = (struct ccci_rpc_usim2nfs *)&tmp_data[1];
			output->lock_vsim1 = input->lock_vsim1;
			CCCI_DEBUG_LOG(md_id, RPC,
				"IPC_RPC_USIM2NFC_OP request=%x\n",
				input->lock_vsim1);
			/* lock_vsim1==1, NFC not power VSIM;
			 * lock_vsim==0, NFC power VSIM
			 */
			inform_nfc_vsim_change(md_id, 1, input->lock_vsim1);
			break;
		}
#endif
	case IPC_RPC_CCCI_LHIF_MAPPING:
		{
			struct ccci_rpc_queue_mapping *remap;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}

			CCCI_NORMAL_LOG(md_id, RPC,
				"op_id[0x%X]: pkt_num=%d, pkt[0] len %u!\n",
				p_rpc_buf->op_id, pkt_num, pkt[0].len);

			remap = (struct ccci_rpc_queue_mapping *)(pkt[0].buf);
			ccci_rpc_remap_queue(md_id, remap);
			pkt_num = 0;
			tmp_data[0] = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];

			break;
		}
	case IPC_RPC_DTSI_QUERY_OP:
		{
			struct ccci_rpc_md_dtsi_input *input;
			struct ccci_rpc_md_dtsi_output *output;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}
			input = (struct ccci_rpc_md_dtsi_input *)(pkt[0].buf);
			pkt_num = 0;
			tmp_data[0] = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len =
				sizeof(struct ccci_rpc_md_dtsi_output);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
			output = (struct ccci_rpc_md_dtsi_output *)&tmp_data[1];
			/* 0xF for failure */
			memset(output, 0xF,
				sizeof(struct ccci_rpc_md_dtsi_output));
			get_md_dtsi_val(input, output);
			break;
		}
	case IPC_RPC_QUERY_CARD_TYPE:
		CCCI_NORMAL_LOG(md_id, RPC,
			"enter QUERY CARD_TYPE operation in ccci_rpc_work\n");
		break;
	case IPC_RPC_TRNG:
		{
			struct arm_smccc_res res = {0};

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d!\n",
					     p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}
			arm_smccc_smc(MTK_SIP_KERNEL_GET_RND,
				TRNG_MAGIC, 0, 0, 0, 0, 0, 0, &res);
			pkt_num = 0;
			tmp_data[0] = 0;
			tmp_data[1] = res.a0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
			break;

		}
	case IPC_RPC_AMMS_DRDI_CONTROL:
		/* PEARL: 内核侧实现 Android ccci_rpcd 的 AMMS DRDI 应答 */
		{
			struct rpc_pkt opkt[RPC_MAX_ARG_NUM];
			int xa_i;

			pkt_num = pearl_amms_handle(port, p_rpc_buf, pkt, pkt_num,
				opkt, (u32 *)tmp_data);
			for (xa_i = 0; xa_i < pkt_num; xa_i++)
				pkt[xa_i] = opkt[xa_i];
		}
		break;
	case IPC_RPC_IT_OP:
		{
			int i;

			CCCI_NORMAL_LOG(md_id, RPC,
				"[RPCIT] enter IT operation in ccci_rpc_work\n");
			/* exam input parameters in pkt */
			for (i = 0; i < pkt_num; i++) {
				CCCI_NORMAL_LOG(md_id, RPC,
					"len=%d val=%X\n", pkt[i].len,
					*((unsigned int *)pkt[i].buf));
			}
			tmp_data[0] = 1;
			tmp_data[1] = 0xA5A5;
			pkt_num = 0;
			CCCI_NORMAL_LOG(md_id, RPC,
				"[RPCIT] prepare output parameters\n");
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			CCCI_NORMAL_LOG(md_id, RPC,
				"[RPCIT] LV[%d]  len= 0x%08X, value= 0x%08X\n",
				0, pkt[0].len, *((unsigned int *)pkt[0].buf));
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
			CCCI_NORMAL_LOG(md_id, RPC,
			"[RPCIT] LV[%d]  len= 0x%08X, value= 0x%08X\n",
			1, pkt[1].len, *((unsigned int *)pkt[1].buf));
			break;
		}

	default:
		CCCI_NORMAL_LOG(md_id, RPC,
		"[Error]Unknown Operation ID (0x%08X)\n",
		p_rpc_buf->op_id);
		tmp_data[0] = FS_NO_OP;
		pkt_num = 0;
		pkt[pkt_num].len = sizeof(int);
		pkt[pkt_num++].buf = (void *)&tmp_data[0];
		break;
	}

	p_rpc_buf->para_num = pkt_num;
	CCCI_DEBUG_LOG(md_id, RPC, "%s-- %d\n", __func__,
		p_rpc_buf->para_num);
}

static void rpc_msg_handler(struct port_t *port, struct sk_buff *skb)
{
	int md_id = port->md_id;
	struct rpc_buffer *rpc_buf = (struct rpc_buffer *)skb->data;
	int i, data_len, AlignLength, ret;
	struct rpc_pkt pkt[RPC_MAX_ARG_NUM];
	char *ptr = NULL, *ptr_base = NULL;
	/* unsigned int tmp_data[128]; */
	/* size of tmp_data should be >= any RPC output result */
	unsigned int *tmp_data =
		kmalloc(128*sizeof(unsigned int), GFP_ATOMIC);

	if (tmp_data == NULL) {
		CCCI_ERROR_LOG(md_id, RPC,
			"RPC request buffer fail 128*sizeof(unsigned int)\n");
		goto err_out;
	}
	/* sanity check */
	if (skb->len > RPC_MAX_BUF_SIZE) {
		CCCI_ERROR_LOG(md_id, RPC,
				"invalid RPC buffer size 0x%x/0x%x\n",
				skb->len, RPC_MAX_BUF_SIZE);
		goto err_out;
	}
	if (rpc_buf->header.reserved < 0 ||
		rpc_buf->header.reserved > RPC_REQ_BUFFER_NUM ||
	    rpc_buf->para_num < 0 ||
		rpc_buf->para_num > RPC_MAX_ARG_NUM) {
		CCCI_ERROR_LOG(md_id, RPC,
			"invalid RPC index %d/%d\n",
			rpc_buf->header.reserved, rpc_buf->para_num);
		goto err_out;
	}
	/* parse buffer */
	ptr_base = ptr = rpc_buf->buffer;
	data_len = sizeof(rpc_buf->op_id) + sizeof(rpc_buf->para_num);
	for (i = 0; i < rpc_buf->para_num; i++) {
		pkt[i].len = *((unsigned int *)ptr);
		if (pkt[i].len >= skb->len) {
			CCCI_ERROR_LOG(md_id, RPC,
				"invalid packet length in parse %u\n",
				pkt[i].len);
			goto err_out;
		}
		if ((data_len + sizeof(pkt[i].len) + pkt[i].len) >
			RPC_MAX_BUF_SIZE) {
			CCCI_ERROR_LOG(md_id, RPC,
				"RPC buffer overflow in parse %zu\n",
				data_len + sizeof(pkt[i].len) + pkt[i].len);
			goto err_out;
		}
		ptr += sizeof(pkt[i].len);
		pkt[i].buf = ptr;
		AlignLength = ((pkt[i].len + 3) >> 2) << 2;
		ptr += AlignLength;	/* 4byte align */
		data_len += (sizeof(pkt[i].len) + AlignLength);
	}
	if ((ptr - ptr_base) > RPC_MAX_BUF_SIZE) {
		CCCI_ERROR_LOG(md_id, RPC,
			"RPC overflow in parse 0x%p\n",
			(void *)(ptr - ptr_base));
		goto err_out;
	}
	/* handle RPC request */
	ccci_rpc_work_helper(port, pkt, rpc_buf, tmp_data);
	/* write back to modem */
	/* update message */
	rpc_buf->op_id |= RPC_API_RESP_ID;
	data_len = sizeof(rpc_buf->op_id) + sizeof(rpc_buf->para_num);
	ptr = rpc_buf->buffer;
	for (i = 0; i < rpc_buf->para_num; i++) {
		if ((data_len + sizeof(pkt[i].len) + pkt[i].len) >
			RPC_MAX_BUF_SIZE) {
			CCCI_ERROR_LOG(md_id, RPC,
				"RPC overflow in write %zu\n",
				data_len + sizeof(pkt[i].len) + pkt[i].len);
			goto err_out;
		}

		*((unsigned int *)ptr) = pkt[i].len;
		ptr += sizeof(pkt[i].len);
		data_len += sizeof(pkt[i].len);
		/* 4byte aligned */
		AlignLength = ((pkt[i].len + 3) >> 2) << 2;
		data_len += AlignLength;

		if (ptr != pkt[i].buf)
			memcpy(ptr, pkt[i].buf, pkt[i].len);
		else
			CCCI_DEBUG_LOG(md_id, RPC,
				"same addr, no copy, op_id=0x%x\n",
				rpc_buf->op_id);

		ptr += AlignLength;
	}
	/* resize skb */
	data_len += sizeof(struct ccci_header);
	if (data_len > skb->len)
		skb_put(skb, data_len - skb->len);
	else if (data_len < skb->len)
		skb_trim(skb, data_len);
	/* update CCCI header */
	rpc_buf->header.channel = CCCI_RPC_TX;
	rpc_buf->header.data[1] = data_len;
	CCCI_DEBUG_LOG(md_id, RPC,
		"Write %d/%d, %08X, %08X, %08X, %08X, op_id=0x%x\n",
		skb->len, data_len, rpc_buf->header.data[0],
		rpc_buf->header.data[1], rpc_buf->header.channel,
		rpc_buf->header.reserved, rpc_buf->op_id);
	/* switch to Tx request */
	ret = port_send_skb_to_md(port, skb, 1);
	if (ret)
		goto err_out;
	kfree(tmp_data);
	return;

 err_out:
	kfree(tmp_data);
	ccci_free_skb(skb);
}

/*
 * define character device operation for rpc_u
 */
static int port_rpc_dev_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct port_t *port = fp->private_data;
	int md_id, len, ret;
	unsigned long pfn;
	struct ccci_smem_region *amms_smem = NULL;

	if (port == NULL) {
		CCCI_ERROR_LOG(-1, RPC, "%s:port is NULL\n", __func__);
		return -1;
	}

	md_id = port->md_id;
	if (port->rx_ch != CCCI_RPC_RX)
		return -EFAULT;

	amms_smem = ccci_md_get_smem_by_user_id(md_id,
		SMEM_USER_MD_DRDI);
	if (!amms_smem) {
		CCCI_ERROR_LOG(md_id, RPC, "%s:%d:ccci_md_get_smem_by_user_id fail\n",
			__func__, __LINE__);
		return -1;
	}

	if (amms_smem->size != BANK4_DRDI_SMEM_SIZE)
		CCCI_ERROR_LOG(md_id, RPC, "%s:%d:SMEM_USER_MD_DRDI size invalid(0x%x)\n",
			__func__, __LINE__, amms_smem->size);
	amms_smem->size &= ~(PAGE_SIZE - 1);
	CCCI_NORMAL_LOG(md_id, RPC,
			"remap drdi smem addr:0x%llx len:%d  map-len:%lx\n",
			(unsigned long long)amms_smem->base_ap_view_phy,
			amms_smem->size, vma->vm_end - vma->vm_start);
	if ((vma->vm_end - vma->vm_start) != amms_smem->size) {
		CCCI_ERROR_LOG(md_id, RPC,
			"smem size error:%s,vm_start=0x%llx,vm_end=0x%llx,smem_size=0x%x\n",
			port->name, vma->vm_start, vma->vm_end, amms_smem->size);
		return -EINVAL;
	}

	len = amms_smem->size;
	pfn = amms_smem->base_ap_view_phy;
	pfn >>= PAGE_SHIFT;
	/* ensure that memory does not get swapped to disk */
	vm_flags_set(vma, VM_IO);
	/* ensure non-cacheable */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	ret = remap_pfn_range(vma, vma->vm_start, pfn,
				len, vma->vm_page_prot);
	if (ret) {
		CCCI_ERROR_LOG(md_id, RPC,
			"drdi_smem remap failed %d/%lx, 0x%llx -> 0x%llx\n",
			ret, pfn,
			(unsigned long long)amms_smem->base_ap_view_phy,
			(unsigned long long)vma->vm_start);
		return -EAGAIN;
	}

	return 0;
}

static const struct file_operations rpc_dev_fops = {
	.owner = THIS_MODULE,
	.open = &port_dev_open, /*use default API*/
	.read = &port_dev_read, /*use default API*/
	.write = &port_dev_write, /*use default API*/
	.release = &port_dev_close,/*use default API*/
	.mmap = &port_rpc_dev_mmap,/*use internal API*/
};
static int port_rpc_init(struct port_t *port)
{
	struct cdev *dev = NULL;
	int ret = 0;
	static int first_init = 1;

	CCCI_DEBUG_LOG(port->md_id, RPC,
		"rpc port %s is initializing\n", port->name);
	port->rx_length_th = MAX_QUEUE_LENGTH;
	port->skb_from_pool = 1;
	port->interception = 0;
	if (port->flags & PORT_F_WITH_CHAR_NODE) {
		dev = kmalloc(sizeof(struct cdev), GFP_KERNEL);
		if (unlikely(!dev)) {
			CCCI_ERROR_LOG(port->md_id, CHAR,
				"alloc rpc char dev fail!!\n");
			return -1;
		}
		cdev_init(dev, &rpc_dev_fops);
		dev->owner = THIS_MODULE;
		ret = cdev_add(dev, MKDEV(port->major,
			port->minor_base + port->minor), 1);
		ret = ccci_register_dev_node(port->name, port->major,
			port->minor_base + port->minor);
		port->flags |= PORT_F_ADJUST_HEADER;
	} else {
		port->skb_handler = &rpc_msg_handler;
		kthread_run(port_kthread_handler, port, "%s", port->name);
	}

	if (first_init) {
		get_dtsi_eint_node(port->md_id);
		get_md_dtsi_debug();
		pearl_drdi_load_image();	/* PEARL: AMMS DRDI copy 的数据源 */
		first_init = 0;
	}
	return 0;
}

int port_rpc_recv_match(struct port_t *port, struct sk_buff *skb)
{
	int md_id = port->md_id;
	int is_userspace_msg = 0;
	struct ccci_header *ccci_h = (struct ccci_header *)skb->data;
	struct rpc_buffer *rpc_buf = (struct rpc_buffer *)skb->data;

	if (ccci_h->channel == CCCI_RPC_RX) {
		switch (rpc_buf->op_id) {
#ifdef CONFIG_MTK_TC1_FEATURE
		/* LGE specific OP ID */
		case RPC_CCCI_LGE_FAC_READ_SIM_LOCK_TYPE:
		case RPC_CCCI_LGE_FAC_READ_FUSG_FLAG:
		case RPC_CCCI_LGE_FAC_CHECK_UNLOCK_CODE_VALIDNESS:
		case RPC_CCCI_LGE_FAC_CHECK_NETWORK_CODE_VALIDNESS:
		case RPC_CCCI_LGE_FAC_WRITE_SIM_LOCK_TYPE:
		case RPC_CCCI_LGE_FAC_READ_IMEI:
		case RPC_CCCI_LGE_FAC_WRITE_IMEI:
		case RPC_CCCI_LGE_FAC_READ_NETWORK_CODE_LIST_NUM:
		case RPC_CCCI_LGE_FAC_READ_NETWORK_CODE:
		case RPC_CCCI_LGE_FAC_WRITE_NETWORK_CODE_LIST_NUM:
		case RPC_CCCI_LGE_FAC_WRITE_UNLOCK_CODE_VERIFY_FAIL_COUNT:
		case RPC_CCCI_LGE_FAC_READ_UNLOCK_CODE_VERIFY_FAIL_COUNT:
		case RPC_CCCI_LGE_FAC_WRITE_UNLOCK_FAIL_COUNT:
		case RPC_CCCI_LGE_FAC_READ_UNLOCK_FAIL_COUNT:
		case RPC_CCCI_LGE_FAC_WRITE_UNLOCK_CODE:
		case RPC_CCCI_LGE_FAC_VERIFY_UNLOCK_CODE:
		case RPC_CCCI_LGE_FAC_WRITE_NETWORK_CODE:
		case RPC_CCCI_LGE_FAC_INIT_SIM_LOCK_DATA:
			is_userspace_msg = 1;
#endif
			break;

		case IPC_RPC_QUERY_AP_SYS_PROPERTY:
		case IPC_RPC_SAR_TABLE_IDX_QUERY_OP:
		case IPC_RPC_SAVE_MD_CAPID:
			is_userspace_msg = 1;
			break;
		case IPC_RPC_AMMS_DRDI_CONTROL:
			/* PEARL: no ccci_rpcd daemon on Mobian; keep DRDI
			 * requests in the kernel RPC handler. */
			is_userspace_msg = 0;
			break;
		default:
			is_userspace_msg = 0;
			break;
		}
	}
	if (is_userspace_msg &&
		(port->flags & PORT_F_WITH_CHAR_NODE)) {
		/*userspace msg, so need match userspace port*/
		CCCI_DEBUG_LOG(md_id, RPC, "userspace rpc msg 0x%x on %s\n",
						rpc_buf->op_id, port->name);
	} else {
		/*kernel msg, so need match kernel port*/
		if (is_userspace_msg == 0 &&
			!(port->flags & PORT_F_WITH_CHAR_NODE)) {
			CCCI_DEBUG_LOG(md_id, RPC,
				"kernelspace rpc msg 0x%x on %s\n",
				rpc_buf->op_id, port->name);
		} else {
			CCCI_DEBUG_LOG(md_id, RPC,
				"port_rpc cfg error, need check:msg 0x%x on %s\n",
				rpc_buf->op_id, port->name);
			return 0;
		}
	}
	return 1;
}

struct port_ops rpc_port_ops = {
	.init = &port_rpc_init,
	.recv_match = &port_rpc_recv_match,
	.recv_skb = &port_recv_skb,
};

