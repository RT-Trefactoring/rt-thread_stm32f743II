/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-09-22     Bernard      add board.h to this bsp
 * 2017-08-25     LongfeiMa    transplantation for stm32h7xx
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stm32h7xx.h>
#include "stm32h7xx_hal.h"


#ifdef __CC_ARM
extern int Image$$RW_IRAM2$$ZI$$Limit;
#define HEAP_BEGIN    (&Image$$RW_IRAM2$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define HEAP_BEGIN    (__segment_end("HEAP"))
#else
extern int __bss_end;
#define HEAP_BEGIN    (&__bss_end)
#endif

#define STM32_SRAM_SIZE   (512 * 1024)
#define HEAP_END          (0x24000000 + STM32_SRAM_SIZE)

void rt_hw_board_init(void);

#endif

