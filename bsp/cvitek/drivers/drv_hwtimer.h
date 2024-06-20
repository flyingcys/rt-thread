/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2024/06/20     ShichengChu    first version
 */
#ifndef __DRV_TIMER_H__
#define __DRV_TIMER_H__

#include "pinctrl.h"
#include "mmio.h"
#define DW_NR_TIMERS 8

#define TIMER_FREQ 25000000

#define DW_TIMER_GET_RELOAD_VAL(_tim_, _frq_)      ((_tim_ < 25000U) ? ((_frq_ * _tim_) / 1000U) : (_frq_ * (_tim_ / 1000U)))

#define DW_TIMER0_BASE              0x030A0000UL
#define DW_TIMER0_SIZE              0x14U

#define DW_TIMER1_BASE              (DW_TIMER0_BASE+DW_TIMER0_SIZE)
#define DW_TIMER1_SIZE              DW_TIMER0_SIZE

#define DW_TIMER2_BASE              (DW_TIMER1_BASE+DW_TIMER1_SIZE)
#define DW_TIMER2_SIZE              DW_TIMER1_SIZE

#define DW_TIMER3_BASE              (DW_TIMER2_BASE+DW_TIMER2_SIZE)
#define DW_TIMER3_SIZE              DW_TIMER2_SIZE

#define DW_TIMER4_BASE              (DW_TIMER3_BASE+DW_TIMER3_SIZE)
#define DW_TIMER4_SIZE              DW_TIMER3_SIZE

#define DW_TIMER5_BASE              (DW_TIMER4_BASE+DW_TIMER4_SIZE)
#define DW_TIMER5_SIZE              DW_TIMER4_SIZE

#define DW_TIMER6_BASE              (DW_TIMER5_BASE+DW_TIMER5_SIZE)
#define DW_TIMER6_SIZE              DW_TIMER5_SIZE

#define DW_TIMER7_BASE              (DW_TIMER6_BASE+DW_TIMER6_SIZE)
#define DW_TIMER7_SIZE              DW_TIMER6_SIZE

#define TIMER_INTR_0 TIMER_IRQ_BASE + 0
#define TIMER_INTR_1 TIMER_IRQ_BASE + 1
#define TIMER_INTR_2 TIMER_IRQ_BASE + 2
#define TIMER_INTR_3 TIMER_IRQ_BASE + 3
#define TIMER_INTR_4 TIMER_IRQ_BASE + 4
#define TIMER_INTR_5 TIMER_IRQ_BASE + 5
#define TIMER_INTR_6 TIMER_IRQ_BASE + 6
#define TIMER_INTR_7 TIMER_IRQ_BASE + 7

/*! Timer1 Control Reg,     offset: 0x08 */
#define DW_TIMER_CTL_ENABLE_SEL_Pos                                    (0U)
#define DW_TIMER_CTL_ENABLE_SEL_Msk                                    (0x1U << DW_TIMER_CTL_ENABLE_SEL_Pos)
#define DW_TIMER_CTL_ENABLE_SEL_EN                                     DW_TIMER_CTL_ENABLE_SEL_Msk

#define DW_TIMER_CTL_MODE_SEL_Pos                                      (1U)
#define DW_TIMER_CTL_MODE_SEL_Msk                                      (0x1U << DW_TIMER_CTL_MODE_SEL_Pos)
#define DW_TIMER_CTL_MODE_SEL_EN                                       DW_TIMER_CTL_MODE_SEL_Msk

#define DW_TIMER_CTL_INT_MASK_Pos                                      (2U)
#define DW_TIMER_CTL_INT_MASK_Msk                                      (0x1U << DW_TIMER_CTL_INT_MASK_Pos)
#define DW_TIMER_CTL_INT_MAKS_EN                                       DW_TIMER_CTL_INT_MASK_Msk

#define DW_TIMER_CTL_HARD_TRIG_Pos                                     (4U)
#define DW_TIMER_CTL_HARD_TRIG_Msk                                     (0x1U << DW_TIMER_CTL_HARD_TRIG_Pos)
#define DW_TIMER_CTL_HARD_TRIG_EN                                      DW_TIMER_CTL_HARD_TRIG_Msk

/*! Timer EOI,            offset: 0x0c */
#define DW_TIMER_EOI_REG_Pos                                           (0U)
#define DW_TIMER_EOI_REG_Msk                                           (0x1U << DW_TIMER_EOI_REG_Pos)
#define DW_TIMER_EOI_REG_EN                                            DW_TIMER_EOI_REG_Msk

/*! Timer Int Status,     offset: 0x10 */
#define DW_TIMER_INT_STATUS_Pos                                        (0U)
#define DW_TIMER_INT_STATUS_Msk                                        (0x1U << DW_TIMER_INT_STATUS_Pos)
#define DW_TIMER_INT_STATUS_EN                                         DW_TIMER_INT_STATUS_Msk

/*! Timers Int Status,    offset: 0xa0 */
#define DW_TIMERS_INT_STATUS_Pos                                       (0U)
#define DW_TIMERS_INT_STATUS_Msk                                       (0x2U << DW_TIMERS_INT_STATUS_Pos)
#define DW_TIMERS_INT_STATUS_EN                                        DW_TIMERS_INT_STATUS_Msk

/*! Timers EOI,           offset: 0xa4 */
#define DW_TIMERS_EOI_REG_Pos                                          (0U)
#define DW_TIMERS_EOI_REG_Msk                                          (0x2U << DW_TIMERS_EOI_REG_Pos)
#define DW_TIMERS_EOI_REG_EN                                           DW_TIMERS_EOI_REG_Msk

/*! Timers Raw Int Status,offset: 0xa8 */
#define DW_TIMERS_RAW_INT_STA_Pos                                      (0U)
#define DW_TIMERS_RAW_INT_STA_Msk                                      (0x2U << DW_TIMERS_RAW_INT_STA_Pos)
#define DW_TIMERS_RAW_INT_STA_EN                                       DW_TIMERS_RAW_INT_STA_Msk

typedef struct {
    volatile uint32_t TLC;                                                /* Offset: 0x000 (R/W) TimerLoadCount */
    volatile const  uint32_t TCV;                                                /* Offset: 0x004 (R/ ) TimerCurrentValue */
    volatile uint32_t TCR;                                                /* Offset: 0x008 (R/W) TimerControlReg */
    volatile const  uint32_t TEOI;                                               /* Offset: 0x00c (R/ ) TimerEOI */
    volatile const  uint32_t TIS;                                                /* Offset: 0x010 (R/ ) TimerIntStatus */
} dw_timer_regs_t;

typedef struct {
    dw_timer_regs_t timer[DW_NR_TIMERS];
    volatile const  uint32_t TSIS;                                               /* Offset: 0x0a0 (R/ ) TimersIntStatus */
    volatile const  uint32_t TSEOI;                                              /* Offset: 0x0a4 (R/ ) TimersEOI */
    volatile const  uint32_t TSRIS;                                              /* Offset: 0x0a8 (R/ ) TimersRawIntStatus */
} dw_timer_general_regs_t;

int rt_hw_hwtimer_init(void);

#endif /* __DRV_TIMER_H__ */
