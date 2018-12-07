/*
 * File      : drv_lcd.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-10-30     Tanek        the first version
 * 2018-04-05     Liu2guang    export LCD config parameters.
 */

#include <rtthread.h>
#include <string.h>
#include <board.h>
#include "drv_ltdc.h"
#include <rthw.h>
#define LCD_BITS_PER_PIXEL          (16)

#define LCD_FRAME_BUFFER_SIZE       (BSP_LCD_WIDTH * BSP_LCD_HEIGHT * (LCD_BITS_PER_PIXEL / 8))

static LTDC_HandleTypeDef hltdc;
static DMA2D_HandleTypeDef hdma2d;
static LTDC_LayerCfgTypeDef pLayerCfg;

enum use_buff { FRONT, BACK };
enum state_buff { EMPTY, FULL };

struct buff_info
{
    rt_uint32_t *buff;
    enum state_buff status;
};

struct drv_lcd
{
    struct rt_device device;
    struct rt_device_graphic_info info;
    struct rt_semaphore lcd_sem;
    int use_screen;
    rt_uint32_t *framebuffer;      //user buff
    struct buff_info front_buf_info;
    struct buff_info back_buf_info;
    enum use_buff current_buf;     //正在使用中的buf  0:front 1:back ...
};

static struct drv_lcd lcd;

void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_LTDC_CLK_ENABLE();
    __HAL_RCC_DMA2D_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    

    GPIO_Initure.Pin=GPIO_PIN_5;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull=GPIO_PULLUP;
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    

    GPIO_Initure.Pin=GPIO_PIN_10; 
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;
    GPIO_Initure.Pull=GPIO_NOPULL;
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Initure.Alternate=GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);
    

    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11;
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);
    

    GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|
                     GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
    

    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|
                     GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10;
    HAL_GPIO_Init(GPIOI,&GPIO_Initure); 
}

static int ltdc_clk_config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkIniture;
    PeriphClkIniture.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    PeriphClkIniture.PLL3.PLL3M = 5;    
    PeriphClkIniture.PLL3.PLL3N = 160;
    PeriphClkIniture.PLL3.PLL3P = 2;
    PeriphClkIniture.PLL3.PLL3Q = 2;  
    PeriphClkIniture.PLL3.PLL3R = 20;
    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkIniture)==HAL_OK)
    {
        return -1;
    }
    return 0;
}

static void lcd_reset_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    __HAL_RCC_GPIOI_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_11, GPIO_PIN_RESET);
    rt_thread_delay(RT_TICK_PER_SECOND / 100);
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_11, GPIO_PIN_SET);
}


/* DMA2D init function */
static void MX_DMA2D_Init(void)
{

    hdma2d.Instance = DMA2D;
    hdma2d.Init.Mode = DMA2D_M2M;
    hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
    hdma2d.Init.OutputOffset = 0;
    hdma2d.LayerCfg[0].InputOffset = 0;
    hdma2d.LayerCfg[0].InputColorMode = DMA2D_INPUT_RGB565;
    hdma2d.LayerCfg[0].AlphaMode = DMA2D_NO_MODIF_ALPHA;
    hdma2d.LayerCfg[0].InputAlpha = 0;
    if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
    {
        RT_ASSERT(RT_NULL);
    }

    if (HAL_DMA2D_ConfigLayer(&hdma2d, 0) != HAL_OK)
    {
        RT_ASSERT(RT_NULL);
    }
}

static void lcd_backlight_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
          
}


static rt_err_t drv_lcd_init(rt_device_t device)
{
    struct drv_lcd * lcd_drv = (struct drv_lcd *)device;
    RT_ASSERT(device != RT_NULL);
    
    ltdc_clk_config();
    lcd_reset_init();
    lcd_backlight_init();
    MX_DMA2D_Init();

    lcd_drv->framebuffer = rt_malloc_align(LCD_FRAME_BUFFER_SIZE, 32);
    memset(lcd_drv->framebuffer, 0, LCD_FRAME_BUFFER_SIZE);
    lcd_drv->front_buf_info.buff = rt_malloc_align(LCD_FRAME_BUFFER_SIZE, 32);
    memset(lcd_drv->front_buf_info.buff, 0, LCD_FRAME_BUFFER_SIZE);
    lcd_drv->back_buf_info.buff = rt_malloc_align(LCD_FRAME_BUFFER_SIZE, 32);
    memset(lcd_drv->back_buf_info.buff, 0, LCD_FRAME_BUFFER_SIZE);

    rt_sem_init(&lcd_drv->lcd_sem, "lcd_sem", 0, RT_IPC_FLAG_FIFO);

    lcd_drv->info.width          = BSP_LCD_WIDTH;
    lcd_drv->info.height         = BSP_LCD_HEIGHT;
    lcd_drv->info.pixel_format   = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    lcd_drv->info.bits_per_pixel = LCD_BITS_PER_PIXEL;
    lcd_drv->info.framebuffer    = (void *)(lcd_drv->framebuffer);
    
    hltdc.Instance = LTDC;
    hltdc.Init.HSPolarity         = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity         = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity         = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity         = LTDC_PCPOLARITY_IPC;
    
    hltdc.Init.Backcolor.Blue     = 0;
    hltdc.Init.Backcolor.Green    = 0;
    hltdc.Init.Backcolor.Red      = 0;
    
    hltdc.Init.HorizontalSync     = BSP_LCD_HSYNC-1;
    hltdc.Init.VerticalSync       = BSP_LCD_VSYNC-1;
    hltdc.Init.AccumulatedHBP     = BSP_LCD_HSYNC + BSP_LCD_HBP-1;
    hltdc.Init.AccumulatedVBP     = BSP_LCD_VSYNC + BSP_LCD_VBP-1;
    hltdc.Init.AccumulatedActiveW = BSP_LCD_WIDTH + BSP_LCD_HSYNC + BSP_LCD_HBP-1;
    hltdc.Init.AccumulatedActiveH = BSP_LCD_HEIGHT + BSP_LCD_VSYNC + BSP_LCD_VBP-1;
    hltdc.Init.TotalWidth         = BSP_LCD_WIDTH + BSP_LCD_HSYNC + BSP_LCD_HBP + BSP_LCD_HFP-1;
    hltdc.Init.TotalHeigh         = BSP_LCD_HEIGHT + BSP_LCD_VSYNC + BSP_LCD_VBP + BSP_LCD_VFP-1;

    RT_ASSERT(HAL_LTDC_Init(&hltdc) == HAL_OK);
    
    
    pLayerCfg.WindowX0 = 0;
    pLayerCfg.WindowX1 = BSP_LCD_WIDTH;
    pLayerCfg.WindowY0 = 0;
    pLayerCfg.WindowY1 = BSP_LCD_HEIGHT;
    pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    pLayerCfg.Alpha = 0xFF;
    pLayerCfg.Alpha0 = 0x00;
    pLayerCfg.BlendingFactor1 = (rt_uint32_t)6<<8;;
    pLayerCfg.BlendingFactor2 = (rt_uint32_t)7<<8;;
    pLayerCfg.FBStartAdress = (uint32_t)lcd_drv->front_buf_info.buff;
    pLayerCfg.ImageWidth = BSP_LCD_WIDTH;
    pLayerCfg.ImageHeight = BSP_LCD_HEIGHT;
    pLayerCfg.Backcolor.Blue = 0;
    pLayerCfg.Backcolor.Green = 0;
    pLayerCfg.Backcolor.Red = 0;
    if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
    {
        RT_ASSERT(RT_NULL);
    }

    HAL_LTDC_SetWindowPosition(&hltdc,0,0,0);
    HAL_LTDC_SetWindowSize(&hltdc,BSP_LCD_WIDTH,BSP_LCD_HEIGHT,0);
    
    __HAL_LTDC_LAYER_ENABLE(&hltdc,0);

    HAL_LTDC_ProgramLineEvent(&hltdc, 0);

    HAL_NVIC_SetPriority(LTDC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LTDC_IRQn);
    
    return RT_EOK;
}

void LTDC_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_LTDC_IRQHandler(&hltdc);
    rt_interrupt_leave();
}

void HAL_LTDC_LineEventCallback(LTDC_HandleTypeDef *hltdc)
{
    struct drv_lcd * lcd_drv = &lcd;
    
    if (lcd_drv->current_buf == FRONT)
    {
        if (lcd_drv->back_buf_info.status == FULL)
        {
            rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, lcd_drv->back_buf_info.buff, LCD_FRAME_BUFFER_SIZE);
            HAL_LTDC_SetAddress(hltdc, (uint32_t)lcd_drv->back_buf_info.buff, 0);
            lcd_drv->front_buf_info.status = EMPTY;
            lcd_drv->current_buf = BACK;
            rt_sem_release(&lcd_drv->lcd_sem);
        }
    }
    else
    {
        if (lcd_drv->front_buf_info.status == FULL)
        {
            rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, lcd_drv->front_buf_info.buff, LCD_FRAME_BUFFER_SIZE);
            HAL_LTDC_SetAddress(hltdc, (uint32_t)lcd_drv->front_buf_info.buff, 0);
            lcd_drv->back_buf_info.status = EMPTY;
            lcd_drv->current_buf = FRONT;
            rt_sem_release(&lcd_drv->lcd_sem);
        }
    }
    __HAL_LTDC_ENABLE_IT(hltdc, LTDC_IT_LI);
}

static rt_err_t drv_lcd_control(rt_device_t device, int cmd, void *args)
{
   static rt_uint32_t memcp_tick = 0;
    struct drv_lcd * lcd_drv = (struct drv_lcd *)device;
    rt_uint32_t memcp_tick_tmp;
    switch(cmd)
    {
    case RTGRAPHIC_CTRL_RECT_UPDATE:
        /* update */
        while ((lcd_drv->front_buf_info.status != EMPTY) && (lcd_drv->back_buf_info.status != EMPTY))
        {
            //wait sem
            rt_sem_take(&lcd_drv->lcd_sem, RT_TICK_PER_SECOND / 20);
        }
        if (lcd_drv->front_buf_info.status == EMPTY)
        {
            memcp_tick_tmp = rt_tick_get();
            memcpy(lcd_drv->front_buf_info.buff, lcd_drv->framebuffer, LCD_FRAME_BUFFER_SIZE);
            memcp_tick = rt_tick_get() - memcp_tick_tmp;
            lcd_drv->front_buf_info.status = FULL;
        }
        else
        {
            //cp to back buffX
            memcp_tick_tmp = rt_tick_get();
            memcpy(lcd_drv->back_buf_info.buff, lcd_drv->framebuffer, LCD_FRAME_BUFFER_SIZE);
            memcp_tick = rt_tick_get() - memcp_tick_tmp;
            lcd_drv->back_buf_info.status = FULL;
        }
        break;

    case RTGRAPHIC_CTRL_POWERON:
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        break;

    case RTGRAPHIC_CTRL_POWEROFF:
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        break;

    case RTGRAPHIC_CTRL_GET_INFO:
        rt_memcpy(args, &lcd_drv->info, sizeof(lcd_drv->info));
        break;

    case RTGRAPHIC_CTRL_SET_MODE:
        break;
    }

    return RT_EOK;
}

int rt_hw_lcd_init(void)
{
    rt_err_t ret;
    
    rt_memset(&lcd, 0x00, sizeof(lcd));
    //stm32_lcd_init(&lcd.device);
    
    lcd.device.type    = RT_Device_Class_Graphic;
    lcd.device.init    = drv_lcd_init;
    lcd.device.open    = RT_NULL;
    lcd.device.close   = RT_NULL;
    lcd.device.read    = RT_NULL;
    lcd.device.write   = RT_NULL;
    lcd.device.control = drv_lcd_control;

    lcd.device.user_data = (void *)&lcd.info;

    ret = rt_device_register(&lcd.device, "lcd", RT_DEVICE_FLAG_RDWR);

    return ret;
}

INIT_DEVICE_EXPORT(rt_hw_lcd_init);
