/*
 * File      : drv_sdram.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2016 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-08-20     xuzhuoyi     The first version for STM32F42x
 */

#ifndef __DRV_SDRAM_H
#define __DRV_SDRAM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <rtthread.h>
#include <board.h>

#define SDRAM_BANK_ADDR                             ((uint32_t)0xC0000000)

#define SDRAM_TIMEOUT                               ((uint32_t)0xFFFF)

#define SDRAM_MODEREG_BURST_LENGTH_1                ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2                ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4                ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8                ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL         ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED        ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2                 ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3                 ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD       ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE        ((uint16_t)0x0200)

#define REFRESH_COUNT                               ((uint32_t)1438)   /* SDRAM refresh counter (90MHz SD clock) */


extern int rt_sdram_hw_init(void);

#ifdef __cplusplus
}
#endif

#endif
