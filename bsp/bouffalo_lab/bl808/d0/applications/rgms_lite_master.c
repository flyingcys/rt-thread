/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022/12/25     flyingcys    first version
 */
#include <rtthread.h>
#include "rpmsg_lite.h"
#include "rpmsg_ns.h"
#include "rpmsg_queue.h"

static volatile struct rpmsg_lite_instance *ipc_rpmsg = NULL;
static volatile rpmsg_queue_handle ipc_queue = NULL;
static volatile struct rpmsg_lite_endpoint *ipc_ept = NULL;

#define XRAM_RINGBUF_ADDR 0x22048000
#define LOCAL_EPT_ADDR                (30U)

int32_t ept_rx_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    rt_kprintf("ept_rx_cb\r\n");

    return 0;
}


void rpmsg_lite_remote_entry(void *param)
{
    rt_kprintf("rpmsg_lite_remote_entry!\r\n");

    ipc_rpmsg = rpmsg_lite_master_init((void *)XRAM_RINGBUF_ADDR, RL_PLATFORM_BL808_LINK_ID, RL_NO_FLAGS);
    
    rpmsg_lite_wait_for_link_up(ipc_rpmsg, RT_WAITING_FOREVER);
    rt_kprintf("link is up!\r\n");

    ipc_queue = rpmsg_queue_create(ipc_rpmsg);
    
    ipc_ept = rpmsg_lite_create_ept(ipc_rpmsg, LOCAL_EPT_ADDR, ept_rx_cb, ipc_queue);

    while (1)
    {
        rt_thread_delay(1000);

    }
}

int rpmsg_lite_remote_steup(void)
{
    rt_thread_t tid = RT_NULL;

    tid = rt_thread_create("rpmsg", rpmsg_lite_remote_entry, RT_NULL, 2048, 10, 20);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    
    return RT_EOK;
}
INIT_ENV_EXPORT(rpmsg_lite_remote_steup);