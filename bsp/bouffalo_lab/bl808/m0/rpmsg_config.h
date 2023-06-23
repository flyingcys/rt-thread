/*
 * Copyright (c) 2011-2021, Shanghai Real-Thread Electronic Technology Co.,Ltd
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-30     pengzongwei  first version
 */
#ifndef __RPMSG_CONFIG_H__
#define __RPMSG_CONFIG_H__

#include <rtthread.h>

#define RL_MS_PER_INTERVAL                          (1)

#define RL_API_HAS_ZEROCOPY                         (1)
#define RL_USE_STATIC_API                           (0)
#define RL_CLEAR_USED_BUFFERS                       (0)
#define RL_USE_MCMGR_IPC_ISR_HANDLER                (0)
#define RL_USE_ENVIRONMENT_CONTEXT                  (0)
#define RL_DEBUG_CHECK_BUFFERS                      (1)
#define RL_ALLOW_CONSUMED_BUFFERS_NOTIFICATION      (0)
#define RL_ASSERT                                   RT_ASSERT

#define VRING_ALIGN                                 (0x1000U)
#define VRING_SIZE                                  (0x10000UL)

#endif
