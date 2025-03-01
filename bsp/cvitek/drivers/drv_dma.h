/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024/06/08     flyingcys    fix transmission failure
 */
#ifndef __DRV_DMA_H__
#define __DRV_DMA_H__

#include "mmio.h"

#include "cvi_dma_ll.h"

#define DW_DMA_BASE                 (0x4330000UL)
#define DMA_CLK_EN_REG		        (0x03002004UL)
#define CLK_SDMA_AXI_BIT	        1

#define DW_DMA_IRQn                 29U


typedef enum {
    DMA_EVENT_TRANSFER_DONE       = 0,  ///< transfer complete
    DMA_EVENT_TRANSFER_HALF_DONE,       ///< transfer half done
    DMA_EVENT_TRANSFER_ERROR,           ///< transfer error
} csi_dma_event_t;

typedef enum {
    DMA_ADDR_INC    = 0,
    DMA_ADDR_DEC,
    DMA_ADDR_CONSTANT
} csi_dma_addr_inc_t;

typedef enum {
    DMA_DATA_WIDTH_8_BITS  = 0,
    DMA_DATA_WIDTH_16_BITS,
    DMA_DATA_WIDTH_32_BITS,
    DMA_DATA_WIDTH_64_BITS,
    DMA_DATA_WIDTH_128_BITS,
    DMA_DATA_WIDTH_512_BITS
} csi_dma_data_width_t;

typedef enum {
    DMA_MEM2MEM     = 0,
    DMA_MEM2PERH,
    DMA_PERH2MEM,
} csi_dma_trans_dir_t;

typedef struct {
    csi_dma_addr_inc_t          src_inc;        ///< source address increment
    csi_dma_addr_inc_t          dst_inc;        ///< destination address increment
    csi_dma_data_width_t        src_tw;         ///< source transfer width in byte
    csi_dma_data_width_t        dst_tw;         ///< destination transfer width in byte
    csi_dma_trans_dir_t         trans_dir;      ///< transfer direction
    uint16_t                    handshake;      ///< handshake id
    uint16_t                    group_len;      ///< group transaction length (unit: bytes)
    uint8_t                     src_reload_en;  ///< 1:dma enable src addr auto reload, 0:disable
    uint8_t                     dst_reload_en;  ///< 1:dma enable dst addr auto reload, 0:disable
    uint8_t                     half_int_en;    ///< 1:dma enable half interrupt, 0: disable
    uint8_t                     lli_src_en;     ///< 1:dma enable llp, 0 disable
    uint8_t                     lli_dst_en;     ///< 1:dma enable llp, 0 disable
} csi_dma_ch_config_t;

#endif /* __DRV_DMA_H__ */
