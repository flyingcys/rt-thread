/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024/05/24     unicornx     first version
 */
#ifndef __DRV_PINMUX_H__
#define __DRV_PINMUX_H__

/*
 * FIXME: At present, we only define the ones we will use, 
 * not all of them. We will need to add them later.
 */
typedef enum _fs_type
{
    fs_none = 0,
    // I2C
    IIC0_SCL,
    IIC0_SDA,
    IIC1_SCL,
    IIC1_SDA,
    IIC2_SCL,
    IIC2_SDA,
    IIC3_SCL,
    IIC3_SDA,
    IIC4_SCL,
    IIC4_SDA,
    // SDI
    SDIO0_CLK,
    SDIO0_CMD,
    SDIO0_D_0,
    // GPIO
    XGPIOA_7,
    XGPIOA_8,
    XGPIOA_9,
    // PWM
    PWM_13,
    PWM_14,
    PWM_15,
    // UART
    UART3_TX,
    //
    EPHY_LNK_LED,
    EPHY_SPD_LED,
    // Debug
    DBG_0,
    DBG_1,
    DBG_2,
    // SPI
    SPI0_SCK,
    SPI0_SDO,
    SPI0_SDI,
    // 
    CAM_MCLK1,
    WG0_D0,
} fs_type;

/*
 * @whitelist: pin name whilelist which is allowed to set. Ignore check if NULL.
 *             NOTE: whitelist should be a string list ended with NULL.
 */
extern void pinmux_config(const char *pin_name, fs_type func_type, const char *whitelist[]);

#endif
