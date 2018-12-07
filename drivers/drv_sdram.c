/*
 * File      : drv_sdram.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2016, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-08-20     xuzhuoyi     The first version for STM32F42x
 * 2017-04-07     lizhen9880   Use SDRAM BANK1
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_sdram.h"
#include <rtdevice.h>
#include "board.h"




/**
    * @brief SDRAM MSP Initialization
    *        This function configures the hardware resources used in this example:
    *           - Peripheral's clock enable
    *           - Peripheral's GPIO Configuration
    * @param hsdram: SDRAM handle pointer
    * @retval None
    */
void HAL_SDRAM_MspInit(SDRAM_HandleTypeDef *hsdram)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    RCC_PeriphCLKInitTypeDef RCCSDRAM_Sture;
    
    //FMC时钟源为PLL2_DIVR2，PLL2_DIVR2=256M   
    RCCSDRAM_Sture.PeriphClockSelection=RCC_PERIPHCLK_FMC;
    RCCSDRAM_Sture.FmcClockSelection=RCC_FMCCLKSOURCE_PLL2;
    RCCSDRAM_Sture.PLL2.PLL2M = 25;
    RCCSDRAM_Sture.PLL2.PLL2N = 332;
    RCCSDRAM_Sture.PLL2.PLL2P = 2;
    RCCSDRAM_Sture.PLL2.PLL2Q = 2;
    RCCSDRAM_Sture.PLL2.PLL2R = 2;
    RCCSDRAM_Sture.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
    RCCSDRAM_Sture.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    RCCSDRAM_Sture.PLL2.PLL2FRACN = 0;
    HAL_RCCEx_PeriphCLKConfig(&RCCSDRAM_Sture); //FMC时钟源为PLL2_DIVR2
    
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO clocks */
    __HAL_RCC_SYSCFG_CLK_ENABLE();				//使能SYSCFG时钟
    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /* Enable FMC clock */
    __HAL_RCC_FMC_CLK_ENABLE();

    /** FMC GPIO Configuration  
    PF0   ------> FMC_A0
    PF1   ------> FMC_A1
    PF2   ------> FMC_A2
    PF3   ------> FMC_A3
    PF4   ------> FMC_A4
    PF5   ------> FMC_A5
    PC0   ------> FMC_SDNWE
    PC2_C   ------> FMC_SDNE0
    PC3_C   ------> FMC_SDCKE0
    PF11   ------> FMC_SDNRAS
    PF12   ------> FMC_A6
    PF13   ------> FMC_A7
    PF14   ------> FMC_A8
    PF15   ------> FMC_A9
    PG0   ------> FMC_A10
    PG1   ------> FMC_A11
    PE7   ------> FMC_D4
    PE8   ------> FMC_D5
    PE9   ------> FMC_D6
    PE10   ------> FMC_D7
    PE11   ------> FMC_D8
    PE12   ------> FMC_D9
    PE13   ------> FMC_D10
    PE14   ------> FMC_D11
    PE15   ------> FMC_D12
    PD8   ------> FMC_D13
    PD9   ------> FMC_D14
    PD10   ------> FMC_D15
    PD14   ------> FMC_D0
    PD15   ------> FMC_D1
    PG2   ------> FMC_A12
    PG4   ------> FMC_BA0
    PG5   ------> FMC_BA1
    PG8   ------> FMC_SDCLK
    PD0   ------> FMC_D2
    PD1   ------> FMC_D3
    PG15   ------> FMC_SDNCAS
    PE0   ------> FMC_NBL0
    PE1   ------> FMC_NBL1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}



/**
    * @brief  Perform the SDRAM exernal memory inialization sequence
    * @param  hsdram: SDRAM handle
    * @param  Command: Pointer to SDRAM command structure
    * @retval None
    */
static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
    __IO uint32_t tmpmrd =0;
    /* Step 3:  Configure a clock configuration enable command */
    Command->CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
    Command->CommandTarget 		    = FMC_SDRAM_CMD_TARGET_BANK1;
    Command->AutoRefreshNumber 	    = 1;
    Command->ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

    /* Step 4: Insert 100 ms delay */
    /* interrupt is not enable, just to delay some time. */
    for (tmpmrd = 0; tmpmrd < 0xfffff; tmpmrd ++)
        ;

    /* Step 5: Configure a PALL (precharge all) command */
    Command->CommandMode            = FMC_SDRAM_CMD_PALL;
    Command->CommandTarget 	        = FMC_SDRAM_CMD_TARGET_BANK1;
    Command->AutoRefreshNumber 	    = 1;
    Command->ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

    /* Step 6 : Configure a Auto-Refresh command */
    Command->CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    Command->CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
    Command->AutoRefreshNumber      = 8;
    Command->ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

    /* Step 7: Program the external memory mode register */
    tmpmrd = (uint32_t) SDRAM_MODEREG_BURST_LENGTH_4          |
                        SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                        SDRAM_MODEREG_CAS_LATENCY_3           |
                        SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                        SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

    Command->CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
    Command->CommandTarget 		    = FMC_SDRAM_CMD_TARGET_BANK1;
    Command->AutoRefreshNumber 	    = 1;
    Command->ModeRegisterDefinition = tmpmrd;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

    /* Step 8: Set the refresh rate counter */
    /* (64ms x Freq)/2^13(row line) - 20 */
    /* Set the device refresh counter */
    HAL_SDRAM_ProgramRefreshRate(hsdram, 64*1000*166/8192-20);//166M
}

/**
    * @brief  Configures the FMC and GPIOs to interface with the SDRAM memory.
    *         This function must be called before any read/write operation
    *         on the SDRAM.
    * @param  None
    * @retval None
    */
int rt_sdram_hw_init(void)
{
    FMC_SDRAM_TimingTypeDef SDRAM_Timing;
    static FMC_SDRAM_CommandTypeDef command;
    static SDRAM_HandleTypeDef hsdram1;
    /*##-1- Configure the SDRAM device #########################################*/
    /* SDRAM device configuration */
    hsdram1.Instance = FMC_SDRAM_DEVICE;

    /* Timing configuration for 166 MHz of PLL2 frequency (332MHz/2) */
    /* TMRD: 2 Clock cycles */
    SDRAM_Timing.LoadToActiveDelay      = 2;
    /* TXSR: min=72ns (12x6.024ns) */
    SDRAM_Timing.ExitSelfRefreshDelay   = 12;
    /* TRAS: min=42ns (7x6.024ns) max=120k (ns) */
    SDRAM_Timing.SelfRefreshTime        = 7;
    /* TRC:  min=60 (10x6.024ns) */
    SDRAM_Timing.RowCycleDelay          = 10;
    /* TWR:  2 Clock cycles */
    SDRAM_Timing.WriteRecoveryTime      = 4;
    /* TRP:  15ns => 3x6.024ns */
    SDRAM_Timing.RPDelay                = 3;
    /* TRCD: 15ns => 15x6.024ns */
    SDRAM_Timing.RCDDelay               = 3;
 
    hsdram1.Init.SDBank                 = FMC_SDRAM_BANK1;
    hsdram1.Init.ColumnBitsNumber       = FMC_SDRAM_COLUMN_BITS_NUM_9;
    hsdram1.Init.RowBitsNumber          = FMC_SDRAM_ROW_BITS_NUM_13;
    hsdram1.Init.MemoryDataWidth        = FMC_SDRAM_MEM_BUS_WIDTH_16;
    hsdram1.Init.InternalBankNumber     = FMC_SDRAM_INTERN_BANKS_NUM_4;
    hsdram1.Init.CASLatency             = FMC_SDRAM_CAS_LATENCY_3;
    hsdram1.Init.WriteProtection        = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
    hsdram1.Init.SDClockPeriod          = FMC_SDRAM_CLOCK_PERIOD_2;
    hsdram1.Init.ReadBurst              = FMC_SDRAM_RBURST_ENABLE;
    hsdram1.Init.ReadPipeDelay          = FMC_SDRAM_RPIPE_DELAY_0;

    /* Initialize the SDRAM controller */
    if(HAL_SDRAM_Init(&hsdram1, &SDRAM_Timing) != HAL_OK)
    {
        return RT_ERROR;
    }

    /* Program the SDRAM external device */
    SDRAM_Initialization_Sequence(&hsdram1, &command);

    return RT_EOK;
}

INIT_BOARD_EXPORT(rt_sdram_hw_init);
