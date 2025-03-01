/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024/02/22     flyingcys    first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_dma.h"
#include "drv_pinmux.h"
#include "drv_ioremap.h"

#define DBG_LEVEL   DBG_LOG
#include <rtdbg.h>
#define LOG_TAG "DRV.DMA"

static void rt_hw_dma_isr(int irqno, void *param)
{

}
#if 0
void rt_hw_dma_ch_start(csi_dma_ch_t *dma_ch, void *srcaddr, void *dstaddr, uint32_t length)
{
	CSI_PARAM_CHK_NORETVAL(dma_ch);

	dma_dbg("csi_dma_ch_start: ctrl_id=%d, ch_id=%d, srcaddr=%p, dstaddr=%p, length=%u\r\n",
			dma_ch->ctrl_id, dma_ch->ch_id, srcaddr, dstaddr, length);

	cvi_dma_ch_start(dma_ch->ctrl_id, dma_ch->ch_id, srcaddr, dstaddr, length);
}
csi_error_t set_dw_config(struct dw_dma_cfg *dw_cfg, csi_dma_ch_config_t *config)
{
	if(config->src_reload_en || config->dst_reload_en){
		dma_err("src/dst reload_en not supported\r\n");
		return CSI_UNSUPPORTED;
	}

	if(config->half_int_en){
		dma_err("half_int_en not supported\r\n");
		return CSI_UNSUPPORTED;
	}

	if(config->src_inc == DMA_ADDR_DEC || config->dst_inc == DMA_ADDR_DEC){
		dma_err("DMA_ADDR_DEC not supported\r\n");
		return CSI_UNSUPPORTED;
	}

	dw_cfg->dst_inc = (cvi_dma_addr_inc_t)config->dst_inc;
	dw_cfg->dst_tw = (cvi_dma_data_width_t)config->dst_tw;
	dw_cfg->group_len = config->group_len;
	dw_cfg->handshake = config->handshake;
	dw_cfg->src_inc = (cvi_dma_addr_inc_t)config->src_inc;
	dw_cfg->src_tw = (cvi_dma_data_width_t)config->src_tw;
	dw_cfg->trans_dir = (cvi_dma_trans_dir_t)config->trans_dir;

	return CSI_OK;
}
rt_err_t rt_hw_dma_ch_config(csi_dma_ch_t *dma_ch, csi_dma_ch_config_t *config)
{
    struct dw_dma_cfg dw_cfg;
    int ret = 0;

    ret = set_dw_config(&dw_cfg, config);
	if(ret)
		return ret;

	cvi_dma_ch_config(dma_ch->ctrl_id, dma_ch->ch_id, &dw_cfg);

    return RT_EOK;
}

void csi_dma_ch_stop(csi_dma_ch_t *dma_ch)
{
	CSI_PARAM_CHK_NORETVAL(dma_ch);

	cvi_dma_ch_stop(dma_ch->ctrl_id, dma_ch->ch_id);
}

void csi_dma_ch_pause(csi_dma_ch_t *dma_ch)
{
	CSI_PARAM_CHK_NORETVAL(dma_ch);

	cvi_dma_ch_pause(dma_ch->ctrl_id, dma_ch->ch_id);
}

void csi_dma_ch_resume(csi_dma_ch_t *dma_ch)
{
	CSI_PARAM_CHK_NORETVAL(dma_ch);

	cvi_dma_ch_resume(dma_ch->ctrl_id, dma_ch->ch_id);
}
rt_err_t rt_hw_dma_ch_start(csi_dma_ch_t *dma_ch)
{
    cvi_dma_ch_start(dma_ch->ctrl_id, dma_ch->ch_id);

    return RT_EOK;
}
#endif

int rt_hw_dma_init(void)
{
    rt_uint8_t i;
    dw_dma_t *dma;

    cvi_dma_init(dma, 0, DW_DMA_BASE, DW_DMA_IRQn);

    rt_hw_interrupt_install(DW_DMA_IRQn, rt_hw_dma_isr, RT_NULL, "dma");    \
    rt_hw_interrupt_umask(DW_DMA_IRQn);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_dma_init);
