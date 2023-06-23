/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * Copyright (c) 2015 Xilinx, Inc.
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 * Copyright 2021 ACRIOS Systems s.r.o.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 * FILE NAME
 *
 *       rpmsg_env_rt-thread.c
 *
 *
 * DESCRIPTION
 *
 *       This file is rt-thread Implementation of env layer for OpenAMP.
 *
 *
 **************************************************************************/

#include <rtthread.h>
#include "rpmsg_compiler.h"
#include "rpmsg_env.h"
#include "rpmsg_platform.h"
#include "virtqueue.h"
#include "rpmsg_lite.h"

#include <stdlib.h>
#include <string.h>

static int32_t env_init_counter       = 0;
static rt_sem_t env_sema     = ((void *)0);
static rt_event_t env_event = ((void *)0);
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
LOCK_STATIC_CONTEXT env_sem_static_context;
struct rt_event env_event_static_context;
#endif

/* RL_ENV_MAX_MUTEX_COUNT is an arbitrary count greater than 'count'
   if the inital count is 1, this function behaves as a mutex
   if it is greater than 1, it acts as a "resource allocator" with
   the maximum of 'count' resources available.
   Currently, only the first use-case is applicable/applied in RPMsg-Lite.
 */
#define RL_ENV_MAX_MUTEX_COUNT (1)

/* Max supported ISR counts */
#define ISR_COUNT (32U)
/*!
 * Structure to keep track of registered ISR's.
 */
struct isr_info
{
    void *data;
};
static struct isr_info isr_table[ISR_COUNT];

#if defined(RL_USE_ENVIRONMENT_CONTEXT) && (RL_USE_ENVIRONMENT_CONTEXT == 1)
#error "This RPMsg-Lite port requires RL_USE_ENVIRONMENT_CONTEXT set to 0"
#endif

/*!
 * env_in_isr
 *
 * @returns - true, if currently in ISR
 *
 */
static int32_t env_in_isr(void)
{
    return platform_in_isr();
}

/*!
 * env_wait_for_link_up
 *
 * Wait until the link_state parameter of the rpmsg_lite_instance is set.
 * Utilize events to avoid busy loop implementation.
 *
 */
uint32_t env_wait_for_link_up(volatile uint32_t *link_state, uint32_t link_id, uint32_t timeout_ms)
{
    // (void)xEventGroupClearBits(event_group, (EventBits_t)(1UL << link_id));
    // if (*link_state != 1U)
    // {
    //     EventBits_t uxBits;
    //     uxBits = xEventGroupWaitBits(event_group, (EventBits_t)(1UL << link_id), pdFALSE, pdTRUE,
    //                                  ((portMAX_DELAY == timeout_ms) ? portMAX_DELAY : timeout_ms / portTICK_PERIOD_MS));
    //     if (uxBits == (EventBits_t)(1UL << link_id))
    //     {
    //         return 1U;
    //     }
    //     else
    //     {
    //         /* timeout */
    //         return 0U;
    //     }
    // }
    // else
    // {
    //     return 1U;
    // }
    rt_kprintf("here: [%d] link_id: [%d]\r\n", __LINE__, link_id);
    rt_uint32_t status;
    rt_int32_t tick;

    if (*link_state != 1U)
    {
        if (env_in_isr() != 0)
        {
            timeout_ms = 0; /* force timeout == 0 when in ISR */
        }

        if (timeout_ms != RT_WAITING_FOREVER)
            tick = timeout_ms / (1000 / RT_TICK_PER_SECOND);
        else
            tick = RT_WAITING_FOREVER;

        if (RT_EOK == rt_event_recv(env_event, (1UL << link_id), RT_EVENT_FLAG_CLEAR | RT_EVENT_FLAG_OR,
                          tick, &status))
        {
            return 1U;
        }
        return 0U;
    }
    else
    {
        return 1U;
    }
}

/*!
 * env_tx_callback
 *
 * Set event to notify task waiting in env_wait_for_link_up().
 *
 */
void env_tx_callback(uint32_t link_id)
{
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // if (env_in_isr() != 0)
    // {
    //     (void)xEventGroupSetBitsFromISR(event_group, (EventBits_t)(1UL << link_id), &xHigherPriorityTaskWoken);
    //     portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    // }
    // else
    // {
    //     (void)xEventGroupSetBits(event_group, (EventBits_t)(1UL << link_id));
    // }
    rt_kprintf("here: [%d] link_id: [%d]\r\n", __LINE__, link_id);

    if (RT_EOK != rt_event_send(env_event, (1UL << link_id)))
    {
        rt_kprintf("rt_evnet_send failed!\r\n");
    }

}

/*!
 * env_init
 *
 * Initializes OS/BM environment.
 *
 */
int32_t env_init(void)
{
    int32_t retval;
    char name[RT_NAME_MAX];
    static uint32_t index = 0;

    // rt_enter_critical(); /* stop scheduler */
    /* verify 'env_init_counter' */
    RL_ASSERT(env_init_counter >= 0);
    if (env_init_counter < 0)
    {
        /* coco begin validated: (env_init_counter < 0) condition will never met unless RAM is corrupted */
        // (void)rt_exit_critical(); /* re-enable scheduler */
        return -1;
        /* coco end */
    }
    env_init_counter++;
    /* multiple call of 'env_init' - return ok */
    if (env_init_counter == 1)
    {
        /* first call */
        rt_kprintf("here: [%04d] %s \r\n", __LINE__, __FUNCTION__);

        memset(name, 0, sizeof(name));
        snprintf(name, sizeof(name), "rlsem-%d", index ++);
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
        // env_sema    = xSemaphoreCreateBinaryStatic(&env_sem_static_context);
        // event_group = xEventGroupCreateStatic(&event_group_static_context);
        if (rt_sem_init(&env_sem_static_context, name, 1, RT_IPC_FLAG_PRIO) != RT_EOK)
        {
            // (void)rt_exit_critical();
            return -1;
        }

        memset(name, 0, sizeof(name));
        snprintf(name, sizeof(name), "rlevent-%d", index ++);
        if (rt_event_init(&env_event_static_context, name, RT_IPC_FLAG_PRIO) != RT_EOK)
        {
            // (void)rt_exit_critical();
            return -1;            
        }
        env_sema = &env_sem_static_context;
        env_event = &env_event_static_context;
#else
        // env_sema    = xSemaphoreCreateBinary();
        // event_group = xEventGroupCreate();
        rt_kprintf("here: [%04d] %s \r\n", __LINE__, __FUNCTION__);
        env_sema = rt_sem_create(name, 0, RT_IPC_FLAG_PRIO);
        if (env_sema == RT_NULL)
        {
            // (void)rt_exit_critical();
            return -1;
        }

        rt_kprintf("here: [%04d] %s \r\n", __LINE__, __FUNCTION__);
        memset(name, 0, sizeof(name));
        snprintf(name, sizeof(name), "rlevent-%d", index ++);
        env_event = rt_event_create(name, RT_IPC_FLAG_PRIO);
        if (env_event == RT_NULL)
        {
            rt_sem_delete(env_sema);
            env_sema = ((void *)0);
            // (void)rt_exit_critical();
            return -1;            
        }
#endif
// #if (configUSE_16_BIT_TICKS == 1)
//         (void)xEventGroupClearBits(event_group, 0xFFu);
// #else
//         (void)xEventGroupClearBits(event_group, 0xFFFFFFu);
// #endif
        (void)memset(isr_table, 0, sizeof(isr_table));
        // (void)rt_exit_critical();
        retval = platform_init();
        // (void)xSemaphoreGive(env_sema);

        return retval;
    }
    else
    {
        // (void)rt_exit_critical();
        /* Get the semaphore and then return it,
         * this allows for platform_init() to block
         * if needed and other tasks to wait for the
         * blocking to be done.
         * This is in ENV layer as this is ENV specific.*/
        // (void)xSemaphoreTake(env_sema, portMAX_DELAY);
        // (void)xSemaphoreGive(env_sema);
        return 0;
    }
}

/*!
 * env_deinit
 *
 * Uninitializes OS/BM environment.
 *
 * @returns - execution status
 */
int32_t env_deinit(void)
{
    int32_t retval;

    rt_enter_critical(); /* stop scheduler */
    /* verify 'env_init_counter' */
    RL_ASSERT(env_init_counter > 0);
    if (env_init_counter <= 0)
    {
        (void)rt_exit_critical(); /* re-enable scheduler */
        return -1;
    }

    /* counter on zero - call platform deinit */
    env_init_counter--;
    /* multiple call of 'env_deinit' - return ok */
    if (env_init_counter <= 0)
    {
        /* last call */
        (void)memset(isr_table, 0, sizeof(isr_table));
        retval = platform_deinit();
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
        rt_event_detach(env_event);
        env_event = ((void *)0);
        rt_sem_detach(env_sema);
        env_sema = ((void *)0);
#else
        rt_event_delete(env_event);
        env_event = ((void *)0);
        rt_sem_delete(env_sema);
        env_sema = ((void *)0);
#endif
        (void)rt_exit_critical();

        return retval;
    }
    else
    {
        (void)rt_exit_critical();
        return 0;
    }
}

/*!
 * env_allocate_memory - implementation
 *
 * @param size
 */
void *env_allocate_memory(uint32_t size)
{
    return rt_malloc(size);
}

/*!
 * env_free_memory - implementation
 *
 * @param ptr
 */
void env_free_memory(void *ptr)
{
    if (ptr != ((void *)0))
    {
        rt_free(ptr);
    }
}

/*!
 *
 * env_memset - implementation
 *
 * @param ptr
 * @param value
 * @param size
 */
void env_memset(void *ptr, int32_t value, uint32_t size)
{
    (void)memset(ptr, value, size);
}

/*!
 *
 * env_memcpy - implementation
 *
 * @param dst
 * @param src
 * @param len
 */
void env_memcpy(void *dst, void const *src, uint32_t len)
{
    (void)memcpy(dst, src, len);
}

/*!
 *
 * env_strcmp - implementation
 *
 * @param dst
 * @param src
 */

int32_t env_strcmp(const char *dst, const char *src)
{
    return (strcmp(dst, src));
}

/*!
 *
 * env_strncpy - implementation
 *
 * @param dest
 * @param src
 * @param len
 */
void env_strncpy(char *dest, const char *src, uint32_t len)
{
    (void)strncpy(dest, src, len);
}

/*!
 *
 * env_strncmp - implementation
 *
 * @param dest
 * @param src
 * @param len
 */
int32_t env_strncmp(char *dest, const char *src, uint32_t len)
{
    return (strncmp(dest, src, len));
}

/*!
 *
 * env_mb - implementation
 *
 */
void env_mb(void)
{
    // MEM_BARRIER();
}

/*!
 * env_rmb - implementation
 */
void env_rmb(void)
{
    // MEM_BARRIER();
}

/*!
 * env_wmb - implementation
 */
void env_wmb(void)
{
    // MEM_BARRIER();
}

/*!
 * env_map_vatopa - implementation
 *
 * @param address
 */
uint32_t env_map_vatopa(void *address)
{
    return platform_vatopa(address);
}

/*!
 * env_map_patova - implementation
 *
 * @param address
 */
void *env_map_patova(uint32_t address)
{
    return platform_patova(address);
}

/*!
 * env_create_mutex
 *
 * Creates a mutex with the given initial count.
 *
 */
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
int32_t env_create_mutex(void **lock, int32_t count, void *context)
#else
int32_t env_create_mutex(void **lock, int32_t count)
#endif
{
    int32_t ret = -1;
    char name[RT_NAME_MAX];
    static uint32_t index = 0;
    if (count > RL_ENV_MAX_MUTEX_COUNT)
    {
        return -1;
    }

    memset(name, 0, sizeof(name));
    snprintf(name, sizeof(name), "rlmutex-%d", index);
    index ++;
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
    // *lock = xSemaphoreCreateCountingStatic((UBaseType_t)RL_ENV_MAX_MUTEX_COUNT, (UBaseType_t)count,
    //                                        (StaticSemaphore_t *)context);
    if (RT_EOK == rt_mutex_init((rt_mutex_t)context, name, RT_IPC_FLAG_PRIO))
    {
        *lock = (void *)context;
        ret = 0;
    }
#else
    // *lock = xSemaphoreCreateCounting((UBaseType_t)RL_ENV_MAX_MUTEX_COUNT, (UBaseType_t)count);
    *lock = rt_mutex_create(name, RT_IPC_FLAG_PRIO);
    if (*lock != ((void *)0))
    {
        ret = 0;
    }
#endif

    return ret;
}

/*!
 * env_delete_mutex
 *
 * Deletes the given lock
 *
 */
void env_delete_mutex(void *lock)
{
    // vSemaphoreDelete(lock);
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
    rt_mutex_detach((rt_mutex_t)lock);
#else
    rt_mutex_delete((rt_mutex_t)lock);
#endif
}

/*!
 * env_lock_mutex
 *
 * Tries to acquire the lock, if lock is not available then call to
 * this function will suspend.
 */
void env_lock_mutex(void *lock)
{
    // SemaphoreHandle_t xSemaphore = (SemaphoreHandle_t)lock;
    if (env_in_isr() == 0)
    {
        rt_kprintf("here: [%04d] %s \r\n", __LINE__, __FUNCTION__);
        // (void)xSemaphoreTake(xSemaphore, portMAX_DELAY);
        rt_mutex_take((rt_mutex_t)lock, RT_WAITING_FOREVER);
    }
}

/*!
 * env_unlock_mutex
 *
 * Releases the given lock.
 */
void env_unlock_mutex(void *lock)
{
    // SemaphoreHandle_t xSemaphore = (SemaphoreHandle_t)lock;
    if (env_in_isr() == 0)
    {
        // (void)xSemaphoreGive(xSemaphore);
        rt_mutex_release((rt_mutex_t)lock);
    }
}

/*!
 * env_create_sync_lock
 *
 * Creates a synchronization lock primitive. It is used
 * when signal has to be sent from the interrupt context to main
 * thread context.
 */
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
int32_t env_create_sync_lock(void **lock, int32_t state, void *context)
{
    return env_create_mutex(lock, state, context); /* state=1 .. initially free */
}
#else
int32_t env_create_sync_lock(void **lock, int32_t state)
{
    return env_create_mutex(lock, state); /* state=1 .. initially free */
}
#endif

/*!
 * env_delete_sync_lock
 *
 * Deletes the given lock
 *
 */
void env_delete_sync_lock(void *lock)
{
    if (lock != ((void *)0))
    {
        env_delete_mutex(lock);
    }
}

/*!
 * env_acquire_sync_lock
 *
 * Tries to acquire the lock, if lock is not available then call to
 * this function waits for lock to become available.
 */
void env_acquire_sync_lock(void *lock)
{
    if (lock != ((void *)0))
    {
        rt_kprintf("here: [%04d] %s \r\n", __LINE__, __FUNCTION__);
        env_lock_mutex(lock);
    }
}

/*!
 * env_release_sync_lock
 *
 * Releases the given lock.
 */
void env_release_sync_lock(void *lock)
{
    if (lock != ((void *)0))
    {
        env_unlock_mutex(lock);
    }
}

/*!
 * env_sleep_msec
 *
 * Suspends the calling thread for given time , in msecs.
 */
void env_sleep_msec(uint32_t num_msec)
{
    // vTaskDelay(num_msec / portTICK_PERIOD_MS);
    rt_thread_mdelay(num_msec);
}

/*!
 * env_register_isr
 *
 * Registers interrupt handler data for the given interrupt vector.
 *
 * @param vector_id - virtual interrupt vector number
 * @param data      - interrupt handler data (virtqueue)
 */
void env_register_isr(uint32_t vector_id, void *data)
{
    RL_ASSERT(vector_id < ISR_COUNT);
    if (vector_id < ISR_COUNT)
    {
        isr_table[vector_id].data = data;
    }
}

/*!
 * env_unregister_isr
 *
 * Unregisters interrupt handler data for the given interrupt vector.
 *
 * @param vector_id - virtual interrupt vector number
 */
void env_unregister_isr(uint32_t vector_id)
{
    RL_ASSERT(vector_id < ISR_COUNT);
    if (vector_id < ISR_COUNT)
    {
        isr_table[vector_id].data = ((void *)0);
    }
}

/*!
 * env_enable_interrupt
 *
 * Enables the given interrupt
 *
 * @param vector_id   - virtual interrupt vector number
 */

void env_enable_interrupt(uint32_t vector_id)
{
    (void)platform_interrupt_enable(vector_id);
}

/*!
 * env_disable_interrupt
 *
 * Disables the given interrupt
 *
 * @param vector_id   - virtual interrupt vector number
 */

void env_disable_interrupt(uint32_t vector_id)
{
    (void)platform_interrupt_disable(vector_id);
}

/*!
 * env_map_memory
 *
 * Enables memory mapping for given memory region.
 *
 * @param pa   - physical address of memory
 * @param va   - logical address of memory
 * @param size - memory size
 * param flags - flags for cache/uncached  and access type
 */

void env_map_memory(uint32_t pa, uint32_t va, uint32_t size, uint32_t flags)
{
    platform_map_mem_region(va, pa, size, flags);
}

/*!
 * env_disable_cache
 *
 * Disables system caches.
 *
 */

void env_disable_cache(void)
{
    platform_cache_all_flush_invalidate();
    platform_cache_disable();
}

/*!
 *
 * env_get_timestamp
 *
 * Returns a 64 bit time stamp.
 *
 *
 */
uint64_t env_get_timestamp(void)
{
    // if (env_in_isr() != 0)
    // {
    //     return (uint64_t)xTaskGetTickCountFromISR();
    // }
    // else
    // {
    //     return (uint64_t)xTaskGetTickCount();
    // }
    return rt_tick_get();
}

/*========================================================= */
/* Util data / functions  */

void env_isr(uint32_t vector)
{
    struct isr_info *info;
    RL_ASSERT(vector < ISR_COUNT);
    if (vector < ISR_COUNT)
    {
        info = &isr_table[vector];
        virtqueue_notification((struct virtqueue *)info->data);
    }
}

/*
 * env_create_queue
 *
 * Creates a message queue.
 *
 * @param queue -  pointer to created queue
 * @param length -  maximum number of elements in the queue
 * @param element_size - queue element size in bytes
 * @param queue_static_storage - pointer to queue static storage buffer
 * @param queue_static_context - pointer to queue static context
 *
 * @return - status of function execution
 */
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
int32_t env_create_queue(void **queue,
                         int32_t length,
                         int32_t element_size,
                         uint8_t *queue_static_storage,
                         rpmsg_static_queue_ctxt *queue_static_context)
{
    int32_t ret = -1;
    char name[RT_NAME_MAX];
    static uint32_t index = 0;

    memset(name, 0, sizeof(name));
    snprintf(name, sizeof(name), "rlmq-%d", index);
    index ++;   
    
    if (RT_EOK == rt_mq_init(queue_static_context, name, queue_static_storage, element_size, length, RT_IPC_FLAG_PRIO))
    {
        *queue = (void *)queue_static_context;
        ret = 0;
    }
        // *queue =
        // xQueueCreateStatic((UBaseType_t)length, (UBaseType_t)element_size, queue_static_storage, queue_static_context);
#else
int32_t env_create_queue(void **queue, int32_t length, int32_t element_size)
{
    // *queue = xQueueCreate((UBaseType_t)length, (UBaseType_t)element_size);
    int32_t ret = -1;
    char name[RT_NAME_MAX];
    static uint32_t index = 0;

    memset(name, 0, sizeof(name));
    snprintf(name, sizeof(name), "rlmq-%d", index);
    index ++;    
    *queue = rt_mq_create(name, element_size, length, RT_IPC_FLAG_PRIO);
    if (*queue != ((void *)0))
    {
        ret = 0;
    }
#endif

    return ret;
}

/*!
 * env_delete_queue
 *
 * Deletes the message queue.
 *
 * @param queue - queue to delete
 */

void env_delete_queue(void *queue)
{
    // vQueueDelete(queue);
    rt_mq_delete(queue);
}

/*!
 * env_put_queue
 *
 * Put an element in a queue.
 *
 * @param queue - queue to put element in
 * @param msg - pointer to the message to be put into the queue
 * @param timeout_ms - timeout in ms
 *
 * @return - status of function execution
 */

int32_t env_put_queue(void *queue, void *msg, uint32_t timeout_ms)
{
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // if (env_in_isr() != 0)
    // {
    //     if (xQueueSendFromISR(queue, msg, &xHigherPriorityTaskWoken) == pdPASS)
    //     {
    //         portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    //         return 1;
    //     }
    // }
    // else
    // {
    //     if (xQueueSend(queue, msg, ((portMAX_DELAY == timeout_ms) ? portMAX_DELAY : timeout_ms / portTICK_PERIOD_MS)) ==
    //         pdPASS)
    //     {
    //         return 1;
    //     }
    // }
    // return 0;

    rt_mq_send(queue, msg, 1);

    return 1;
}

/*!
 * env_get_queue
 *
 * Get an element out of a queue.
 *
 * @param queue - queue to get element from
 * @param msg - pointer to a memory to save the message
 * @param timeout_ms - timeout in ms
 *
 * @return - status of function execution
 */

int32_t env_get_queue(void *queue, void *msg, uint32_t timeout_ms)
{
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // if (env_in_isr() != 0)
    // {
    //     if (xQueueReceiveFromISR(queue, msg, &xHigherPriorityTaskWoken) == pdPASS)
    //     {
    //         portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    //         return 1;
    //     }
    // }
    // else
    // {
    //     if (xQueueReceive(queue, msg,
    //                       ((portMAX_DELAY == timeout_ms) ? portMAX_DELAY : timeout_ms / portTICK_PERIOD_MS)) == pdPASS)
    //     {
    //         return 1;
    //     }
    // }
    // return 0;
    if (env_in_isr() != 0)
    {
        timeout_ms = 0; /* force timeout == 0 when in ISR */
    }
    rt_mq_recv(queue, msg, 1, timeout_ms);

    return 1;
}

/*!
 * env_get_current_queue_size
 *
 * Get current queue size.
 *
 * @param queue - queue pointer
 *
 * @return - Number of queued items in the queue
 */

int32_t env_get_current_queue_size(void *queue)
{
    // if (env_in_isr() != 0)
    // {
    //     return ((int32_t)uxQueueMessagesWaitingFromISR(queue));
    // }
    // else
    // {
    //     return ((int32_t)uxQueueMessagesWaiting(queue));
    // }

    return -1;
}
