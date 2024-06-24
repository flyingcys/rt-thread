/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024/01/11     flyingcys    The first version
 */

#include <rthw.h>
#include <rtthread.h>

#include <encoding.h>
#include "sbi.h"
#include "tick.h"

static volatile rt_uint64_t time_elapsed = 0;
static volatile unsigned long tick_cycles = 0;

static rt_uint64_t get_ticks(void)
{
    __asm__ __volatile__(
        "rdtime %0"
        : "=r"(time_elapsed));
    return time_elapsed;
}

int rt_hw_tick_isr(void)
{
    rt_tick_increase();
    sbi_set_timer(get_ticks() + tick_cycles);
    return 0;
}

/* Sets and enable the timer interrupt */
int rt_hw_tick_init(void)
{
    /* Clear the Machine-Timer bit in MIE */
    clear_csr(sie, SIP_STIP);

    tick_cycles = TIMER_CLK_FREQ / RT_TICK_PER_SECOND;

    sbi_set_timer(get_ticks() + tick_cycles);

    rt_kprintf("[rt_hw_tick_init] time_elapsed: %d tick_cycles:%d\n", time_elapsed, tick_cycles);

    /* Enable the Machine-Timer bit in MIE */
    set_csr(sie, SIP_STIP);

    return 0;
}

/**
 * This function will delay for some us.
 *
 * @param us the delay time of us
 */
void rt_hw_us_delay(rt_uint32_t us)
{
    unsigned long start_time;
    unsigned long end_time;
    unsigned long run_time;

    start_time = get_ticks();
    end_time = start_time + us * (TIMER_CLK_FREQ / 1000000);
    do{
        run_time = get_ticks();
    } while(run_time < end_time);
}
