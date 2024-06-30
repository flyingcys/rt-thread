/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2024/06/20     ShichengChu    first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_hwtimer.h"

#define DBG_LEVEL   DBG_LOG
#include <rtdbg.h>
#define LOG_TAG "DRV.HWTIMER"

typedef struct _timer
{
    char *name;
    dw_timer_regs_t *base;
    rt_uint32_t irqno;
    rt_hwtimer_t timer;
}_timer_t;

static void _hwtimer_init(rt_hwtimer_t *timer, rt_uint32_t state);
static rt_err_t _hwtimer_start(rt_hwtimer_t *timer, rt_uint32_t cnt, rt_hwtimer_mode_t mode);
static void _hwtimer_stop(rt_hwtimer_t *timer);
static rt_uint32_t _hwtimer_count_get(rt_hwtimer_t *timer);
static rt_err_t _hwtimer_control(rt_hwtimer_t *timer, rt_uint32_t cmd, void *args);

static const struct rt_hwtimer_ops _hwtimer_ops = {
    .init = _hwtimer_init,
    .start = _hwtimer_start,
    .stop = _hwtimer_stop,
    .count_get = _hwtimer_count_get,
    .control = _hwtimer_control
};

static const struct rt_hwtimer_info _hwtimer_info = {
    .maxfreq = 25000000UL,
    .minfreq = 25000000UL,
    .maxcnt = 0xFFFFFFFF,
    .cntmode = HWTIMER_MODE_PERIOD
};

static _timer_t _timer_obj[] =
{
#ifdef BSP_USING_TIMER0
    {
        .name = "timer0",
        .base = DW_TIMER0_BASE,
        .irqno = TIMER_INTR_0
    },
#endif /* BSP_USING_TIMER0 */
#ifdef BSP_USING_TIMER1
    {
        .name = "timer1",
        .base = DW_TIMER1_BASE,
        .irqno = TIMER_INTR_1
    },
#endif /* BSP_USING_TIMER1 */
#ifdef BSP_USING_TIMER2
    {
        .name = "timer2",
        .base = DW_TIMER2_BASE,
        .irqno = TIMER_INTR_2
    },
#endif /* BSP_USING_TIMER2 */
#ifdef BSP_USING_TIMER3
    {
        .name = "timer3",
        .base = DW_TIMER3_BASE,
        .irqno = TIMER_INTR_3
    },
#endif /* BSP_USING_TIMER3 */
#ifdef BSP_USING_TIMER4
    {
        .name = "timer4",
        .base = DW_TIMER4_BASE,
        .irqno = TIMER_INTR_4
    },
#endif /* BSP_USING_TIMER4 */
#ifdef BSP_USING_TIMER5
    {
        .name = "timer5",
        .base = DW_TIMER5_BASE,
        .irqno = TIMER_INTR_5
    },
#endif /* BSP_USING_TIMER5 */
#ifdef BSP_USING_TIMER6
    {
        .name = "timer6",
        .base = DW_TIMER6_BASE,
        .irqno = TIMER_INTR_6
    },
#endif /* BSP_USING_TIMER6 */
#ifdef BSP_USING_TIMER7
    {
        .name = "timer7",
        .base = DW_TIMER7_BASE,
        .irqno = TIMER_INTR_7
    },
#endif /* BSP_USING_TIMER7 */
};

uint32_t hal_timer_read_load(dw_timer_regs_t *timer_base)
{
    return (timer_base->TLC);
}
void hal_timer_write_load(dw_timer_regs_t *timer_base, uint32_t value)
{
    timer_base->TLC = value;
}
uint32_t hal_timer_get_current(dw_timer_regs_t *timer_base)
{
    return (timer_base->TCV);
}
void hal_timer_set_enable(dw_timer_regs_t *timer_base)
{
    timer_base->TCR |= (DW_TIMER_CTL_ENABLE_SEL_EN);
}
void hal_timer_set_disable(dw_timer_regs_t *timer_base)
{
    timer_base->TCR &= ~(DW_TIMER_CTL_ENABLE_SEL_EN);
}
uint32_t hal_timer_get_enable(dw_timer_regs_t *timer_base)
{
    return (((timer_base->TCR) & DW_TIMER_CTL_ENABLE_SEL_EN) ? (uint32_t)1 : (uint32_t)0);
}
void hal_timer_set_mode_free(dw_timer_regs_t *timer_base)
{
    timer_base->TCR &= ~(DW_TIMER_CTL_MODE_SEL_EN);
}
void hal_timer_set_mode_load(dw_timer_regs_t *timer_base)
{
    timer_base->TCR |= (DW_TIMER_CTL_MODE_SEL_EN);
}
uint32_t hal_timer_get_model(dw_timer_regs_t *timer_base)
{
    return (((timer_base->TCR) & DW_TIMER_CTL_MODE_SEL_EN) ? (uint32_t)1 : (uint32_t)0);
}
void hal_timer_set_mask(dw_timer_regs_t *timer_base)
{
    timer_base->TCR |= (DW_TIMER_CTL_INT_MAKS_EN);
}
void hal_timer_set_unmask(dw_timer_regs_t *timer_base)
{
    timer_base->TCR &= ~(DW_TIMER_CTL_INT_MAKS_EN);
}
uint32_t hal_timer_get_mask(dw_timer_regs_t *timer_base)
{
    return (((timer_base->TCR) & DW_TIMER_CTL_INT_MAKS_EN) ? (uint32_t)1 : (uint32_t)0);
}
void hal_timer_set_hardtrigger_en(dw_timer_regs_t *timer_base)
{
    timer_base->TCR |= (DW_TIMER_CTL_HARD_TRIG_EN);
}
void hal_timer_set_hardtrigger_dis(dw_timer_regs_t *timer_base)
{
    timer_base->TCR &= ~(DW_TIMER_CTL_HARD_TRIG_EN);
}
uint32_t hal_timer_get_hardtrigger(dw_timer_regs_t *timer_base)
{
    return (((timer_base->TCR) & DW_TIMER_CTL_HARD_TRIG_EN) ? (uint32_t)1 : (uint32_t)0);
}
uint32_t hal_timer_clear_irq(dw_timer_regs_t *timer_base)
{
    return (((timer_base->TEOI) & DW_TIMER_EOI_REG_EN) ? (uint32_t)1 : (uint32_t)0);
}
uint32_t hal_timer_get_int_status(dw_timer_regs_t *timer_base)
{
    return (((timer_base->TIS) & DW_TIMER_INT_STATUS_EN) ? (uint32_t)1 : (uint32_t)0);
}
void hal_timer_reset_register(dw_timer_regs_t *timer_base)
{
    timer_base->TCR = 0U;
    timer_base->TLC = 0U;
}
uint32_t hal_timer_general_active_after_mask(dw_timer_general_regs_t *timer_base)
{
    return ((timer_base->TSIS) & DW_TIMERS_INT_STATUS_EN);
}
uint32_t hal_timer_general_clear_irq(dw_timer_general_regs_t *timer_base)
{
    return ((timer_base->TSEOI) & DW_TIMERS_EOI_REG_EN);
}
uint32_t hal_timer_general_active_prior_mask(dw_timer_general_regs_t *timer_base)
{
    return ((timer_base->TSRIS) & DW_TIMERS_RAW_INT_STA_EN);
}

static void rt_hw_hwtmr_isr(int irqno, void *param)
{
    _timer_t *_tmr = param;
    dw_timer_regs_t *timer_base = _tmr->base;

    if (hal_timer_get_int_status(timer_base))
    {
        hal_timer_clear_irq(timer_base);
        hal_timer_set_disable(timer_base);

        rt_device_hwtimer_isr(&_tmr->timer);
        if(_tmr->timer.mode == HWTIMER_MODE_PERIOD)
        {
            hal_timer_set_enable(timer_base);
            hal_timer_set_unmask(timer_base);
            // rt_hw_interrupt_umask(_tmr->irqno);
        }
    }
}

static void _hwtimer_init(rt_hwtimer_t *timer, rt_uint32_t state)
{
    _timer_t *_tmr = rt_container_of(timer, _timer_t, timer);

    RT_ASSERT(_tmr!=NULL)
    if(state)
    {
        hal_timer_reset_register(_tmr->base);
    }
}

static rt_err_t _hwtimer_start(rt_hwtimer_t *timer, rt_uint32_t cnt, rt_hwtimer_mode_t mode)
{
    _timer_t *_tmr = rt_container_of(timer, _timer_t, timer);
    uint32_t tmp_freq = TIMER_FREQ / 1000U;
    uint32_t tmp_load = cnt;

    hal_timer_set_mode_load(_tmr->base);

    //FIXME: no less than 10
    if (tmp_load < 10) {
        tmp_load = 10;
    }

    hal_timer_set_disable(_tmr->base);
    hal_timer_write_load(_tmr->base, tmp_load);

    hal_timer_set_enable(_tmr->base);
    hal_timer_set_unmask(_tmr->base);

    return RT_EOK;
}

static void _hwtimer_stop(rt_hwtimer_t *timer)
{
    _timer_t *_tmr = rt_container_of(timer, _timer_t, timer);

    hal_timer_set_mask(_tmr->base);
    hal_timer_set_disable(_tmr->base);
}

static rt_uint32_t _hwtimer_count_get(rt_hwtimer_t *timer)
{
    _timer_t *_tmr = rt_container_of(timer, _timer_t, timer);
    rt_uint32_t cnt = hal_timer_get_current(_tmr->base);

    return cnt;
}

static rt_err_t _hwtimer_control(rt_hwtimer_t *timer, rt_uint32_t cmd, void *args)
{
    rt_err_t err = RT_EOK;
    _timer_t *_tmr = rt_container_of(timer, _timer_t, timer);

    switch (cmd)
    {
    case HWTIMER_CTRL_FREQ_SET:
        err = -RT_ERROR;
        break;
    case HWTIMER_CTRL_INFO_GET:
        *(rt_hwtimer_t*)args = _tmr->timer;
        break;
    case HWTIMER_CTRL_MODE_SET:
        _tmr->timer.mode = *(rt_uint32_t*)args;
        break;
    case HWTIMER_CTRL_STOP:
        _hwtimer_stop(timer);
        break;
    }

    return err;
}

int rt_hw_hwtimer_init(void)
{
    int ret = RT_EOK;

    for (uint32_t i = 0; i < sizeof(_timer_obj) / sizeof(_timer_obj[0]); i++)
    {
        _timer_obj[i].timer.info = &_hwtimer_info;
        _timer_obj[i].timer.ops = &_hwtimer_ops;
        ret = rt_device_hwtimer_register(&_timer_obj[i].timer, _timer_obj[i].name, &_timer_obj[i]);
        if (ret != RT_EOK)
        {
            LOG_E("%s register failed", _timer_obj[i].name);
        }
        rt_hw_interrupt_install(_timer_obj[i].irqno, rt_hw_hwtmr_isr, &_timer_obj[i], _timer_obj[i].name);
        rt_hw_interrupt_umask(_timer_obj[i].irqno);
    }

    return ret;
}

INIT_DEVICE_EXPORT(rt_hw_hwtimer_init);
