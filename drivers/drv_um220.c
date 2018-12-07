#include <stdio.h>
#include <string.h>

#include <rthw.h>


#include "board.h"
#include <stm32h7xx.h>

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>

/* serial config for um220 */
struct serial_configure um220_config = 
    {
        BAUD_RATE_9600,   /* 9600 bits/s */
        DATA_BITS_8,      /* 8 databits */
        STOP_BITS_1,      /* 1 stopbit */
        PARITY_NONE,      /* No parity  */ 
        BIT_ORDER_LSB,    /* LSB first sent */
        NRZ_NORMAL,       /* Normal mode */
        1024,             /* Buffer size */
        0   
    };
/*****************************************************/

#define UM220_FIFO_SIZE                      512

static struct rt_semaphore um220_rx_notice;
static struct rt_ringbuffer *um220_rx_fifo = RT_NULL;



static char um220_getchar(void)
{
    char ch;

    rt_sem_take(&um220_rx_notice, RT_WAITING_FOREVER);
    rt_ringbuffer_getchar(um220_rx_fifo, (rt_uint8_t *)&ch);

    return ch;
}

static void um220_entry(void *param)
{
    char ch;

    while(1)
    {
        ch = um220_getchar();
		//rt_kprintf("%c",ch);
    }
}

static rt_err_t um220_getchar_rx_ind(rt_device_t dev, rt_size_t size)
{
    uint8_t ch;
    rt_size_t i;

    for (i = 0; i < size; i++)
    {
        /* read a char */
        if (rt_device_read(dev, 0, &ch, 1))
        {
            rt_ringbuffer_put_force(um220_rx_fifo, &ch, 1);
            rt_sem_release(&um220_rx_notice);
        }
    }

    return RT_EOK;
}

static int rt_um220_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//UM220控制部分
	//UM220_RESET,为高
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Configure GPIO pin: PA4 */
	GPIO_InitStruct.Pin   = GPIO_PIN_4;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	rt_thread_delay(20);//>5ms复位
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	
    rt_base_t int_lvl;
    rt_device_t um220_device;
	rt_thread_t um220_thread;

    rt_sem_init(&um220_rx_notice, "um220_notice", 0, RT_IPC_FLAG_FIFO);

    /* create RX FIFO */
    um220_rx_fifo = rt_ringbuffer_create(UM220_FIFO_SIZE);
    /* created must success */
    RT_ASSERT(um220_rx_fifo);

    int_lvl = rt_hw_interrupt_disable();
	
	um220_device = rt_device_find("uart7");
	
    if (um220_device)
    {
	
		rt_device_open(um220_device , RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_INT_RX);
		
        //rt_device_control(um220_device, RT_DEVICE_CTRL_CONFIG,(void *)&um220_config);
		
		rt_device_set_rx_indicate(um220_device, um220_getchar_rx_ind);	
		
    }

    rt_hw_interrupt_enable(int_lvl);
	
	um220_thread = rt_thread_create("um220",um220_entry, RT_NULL, 512, 9, 5);
	
	if (um220_thread!= RT_NULL)		
		rt_thread_startup(um220_thread);
	  return 0;	
}

//INIT_APP_EXPORT(rt_um220_init);
