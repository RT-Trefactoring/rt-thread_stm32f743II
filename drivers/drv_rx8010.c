/*
 * File      : drv_iic.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2017 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-06-09     tanek        first implementation.
 */
#include "drv_rx8010.h"
#include <board.h>
#include <rthw.h>
#include <finsh.h>
#include <rtdevice.h>
#if defined(RT_USING_RTC)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef RT_RTC_DEBUG
#define rtc_debug(format,args...) 			rt_kprintf(format, ##args)
#else
#define rtc_debug(format,args...)
#endif

#define I2C_BUS_NAME	"i2c0"

#define RX8010_ADDR 	0x32


#define RX8010_SEC     0x10
#define RX8010_MIN     0x11
#define RX8010_HOUR    0x12
#define RX8010_WDAY    0x13
#define RX8010_MDAY    0x14
#define RX8010_MONTH   0x15
#define RX8010_YEAR    0x16
#define RX8010_YEAR    0x16
#define RX8010_RESV17  0x17
#define RX8010_ALMIN   0x18
#define RX8010_ALHOUR  0x19
#define RX8010_ALWDAY  0x1A
#define RX8010_TCOUNT0 0x1B
#define RX8010_TCOUNT1 0x1C
#define RX8010_EXT     0x1D
#define RX8010_FLAG    0x1E
#define RX8010_CTRL    0x1F
/* 0x20 to 0x2F are user registers */
#define RX8010_RESV30  0x30
#define RX8010_RESV31  0x31
#define RX8010_IRQ     0x32

#define RX8010_EXT_WADA  BIT(3)

#define RX8010_FLAG_VLF  BIT(1)
#define RX8010_FLAG_AF   BIT(3)
#define RX8010_FLAG_TF   BIT(4)
#define RX8010_FLAG_UF   BIT(5)

#define RX8010_CTRL_AIE  BIT(3)
#define RX8010_CTRL_UIE  BIT(5)
#define RX8010_CTRL_STOP BIT(6)
#define RX8010_CTRL_TEST BIT(7)

#define RX8010_ALARM_AE  BIT(7)


static struct rt_i2c_bus_device * i2c_bus;
/* Private variables ---------------------------------------------------------*/
static struct rt_device rtc;
// write a 8 bit value to a register pair

static rt_err_t rx8010_writebyte(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t send_buf[2];

	send_buf[0] = reg;
	send_buf[1] = (data & 0xff);
	
	
	rt_device_write(&i2c_bus->parent, addr, &send_buf, 2);

    return RT_EOK;
}
//read a 16 bit value from a register

static rt_uint8_t rx8010_readbyte(rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t data)
{

	rt_uint8_t buf1[1];
	rt_uint8_t buf2[1];
	buf1[0] = reg;
	
	rt_device_write(&i2c_bus->parent, addr, &buf1, 1);
	
	
	rt_device_read(&i2c_bus->parent, addr, &buf2, 1);
	 
    data =buf2[0]; 
	
	return data;
}

int rx8010_get_time()
{
	
	struct tm tm_new;
	rt_uint8_t sec,min,hour,wday,mday,mon,year;
	
	tm_new.tm_sec  = rx8010_readbyte(RX8010_ADDR,RX8010_SEC,sec); 
    tm_new.tm_min  = rx8010_readbyte(RX8010_ADDR,RX8010_MIN,min); 
    tm_new.tm_hour = rx8010_readbyte(RX8010_ADDR,RX8010_HOUR,hour); 
	
	tm_new.tm_mday = rx8010_readbyte(RX8010_ADDR,RX8010_MDAY,mday);
    tm_new.tm_mon  = rx8010_readbyte(RX8010_ADDR,RX8010_MONTH,mon); 
    tm_new.tm_year = rx8010_readbyte(RX8010_ADDR,RX8010_YEAR,year);
	
    tm_new.tm_wday = rx8010_readbyte(RX8010_ADDR,RX8010_WDAY,wday); 

    return mktime(&tm_new);
}

int rx8010_set_time(time_t time_stamp)
{

	struct tm *p_tm;
    p_tm = localtime(&time_stamp);
    if(p_tm->tm_year<100)
    {
        return RT_ERROR;
    }

	rx8010_writebyte(RX8010_ADDR,RX8010_SEC, p_tm->tm_sec);
	rx8010_writebyte(RX8010_ADDR,RX8010_MIN, p_tm->tm_min);
	rx8010_writebyte(RX8010_ADDR,RX8010_HOUR,p_tm->tm_hour);
	
	rx8010_writebyte(RX8010_ADDR,RX8010_MDAY,p_tm->tm_mday);
	rx8010_writebyte(RX8010_ADDR,RX8010_MONTH,p_tm->tm_mon);
	rx8010_writebyte(RX8010_ADDR,RX8010_YEAR,p_tm->tm_year);

	rx8010_writebyte(RX8010_ADDR,RX8010_WDAY,p_tm->tm_wday);
	
    return RT_EOK;
}

rt_err_t rx8010_config()
{
	rx8010_writebyte(RX8010_ADDR,RX8010_RESV17, 0xD8);
	rx8010_writebyte(RX8010_ADDR,RX8010_RESV30, 0x00);
	rx8010_writebyte(RX8010_ADDR,RX8010_RESV31, 0x08);
	rx8010_writebyte(RX8010_ADDR,RX8010_IRQ, 0x00);

    return RT_EOK;
}

int rt_rx8010_init(void)
{

	

	i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(I2C_BUS_NAME);
	if (i2c_bus == RT_NULL)
	{
		rtc_debug("\ni2c_bus %s for rx8010 not found!\n", I2C_BUS_NAME);
		return -RT_ENOSYS;
	}

	if (rt_device_open(&i2c_bus->parent, RT_NULL) != RT_EOK)
	{
		rtc_debug("\ni2c_bus %s for rx8010 opened failed!\n", I2C_BUS_NAME);
		return -RT_EEMPTY;
	}

	rx8010_config();
	
	return 0;
}
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static rt_err_t rt_rtc_open(rt_device_t dev, rt_uint16_t oflag)
{
    if (dev->rx_indicate != RT_NULL)
    {
        /* Open Interrupt */
    }

    return RT_EOK;
}

static rt_size_t rt_rtc_read(
	rt_device_t 	dev,
	rt_off_t 		pos,
	void* 			buffer,
	rt_size_t 		size)
{
    
    return 0;
}

/***************************************************************************//**
 * @brief
 *  Configure RTC device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *  Pointer to device descriptor
 *
 * @param[in] cmd
 *  RTC control command
 *
 * @param[in] args
 *  Arguments
 *
 * @return
 *  Error code
 ******************************************************************************/
static rt_err_t rt_rtc_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result;
    RT_ASSERT(dev != RT_NULL);
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
       
        *(rt_uint32_t *)args = rx8010_get_time();
		rtc_debug("RTC: get rtc_time %x\n", *(rt_uint32_t *)args());
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
    {
        result = rx8010_set_time(*(rt_uint32_t *)args);
		rtc_debug("RTC: set rtc_time %x\n", *(rt_uint32_t *)args);

		/* Reset counter */

    }
    break;
    }

    return result;
}



/***************************************************************************//**
 * @brief
 *  Register RTC device
 *
 * @details
 *
 * @note
 *
 * @param[in] device
 *  Pointer to device descriptor
 *
 * @param[in] name
 *  Device name
 *
 * @param[in] flag
 *  Configuration flags
 *
 * @return
 *  Error code
 ******************************************************************************/
rt_err_t rt_hw_rtc_register(
	rt_device_t		device,
	const char		*name,
	rt_uint32_t		flag)
{
	RT_ASSERT(device != RT_NULL);

	device->type 		= RT_Device_Class_RTC;
	device->rx_indicate = RT_NULL;
	device->tx_complete = RT_NULL;
	device->init 		= RT_NULL;
	device->open		= rt_rtc_open;
	device->close		= RT_NULL;
	device->read 		= rt_rtc_read;
	device->write 		= RT_NULL;
	device->control 	= rt_rtc_control;
	device->user_data	= RT_NULL; /* no private */

	/* register a character device */
	return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR | flag);
}


/***************************************************************************//**
 * @brief
 *  Initialize all RTC module related hardware and register RTC device to kernel
 *
 * @details
 *
 * @note
 ******************************************************************************/
int  rt_hw_rtc_init(void)
{
	rt_rx8010_init();
    /* register rtc device */
	rt_hw_rtc_register(&rtc, RT_RTC_NAME, 0);
	
    return RT_EOK;
}

INIT_APP_EXPORT(rt_hw_rtc_init);

#endif
