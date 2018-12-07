/*
 * File      : drv_ft5x06.c
 *             ft5x06 touch driver
 * COPYRIGHT (C) 2006 - 2017, RT-Thread Development Team
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
 * 2017-08-08     Yang        the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "drv_touch.h"
#include "string.h"


#define DBG_ENABLE
#define DBG_SECTION_NAME  "TOUCH.ft5x06"
#define DBG_LEVEL         TOUCH_DBG_LEVEL
#define DBG_COLOR
#include <rtdbg.h>

#define FT5x06_TS_ADDR        (0x38)
#define POINT_SIZE            (7)

#ifdef BSP_USING_TOUCH

struct ponit_info
{
    unsigned char  u8ID;
    unsigned short u16PosX;     // coordinate X, plus 4 LSBs for precision extension
    unsigned short u16PosY;     // coordinate Y, plus 4 LSBs for precision extension
    unsigned char  u8Pressure;
    unsigned char  u8EventId;
};

typedef struct ponit_info ponit_info_t;


static struct rt_i2c_bus_device *ft5x06_i2c_bus;
static struct touch_drivers ft5x06_driver;

static int ft5x06_read(unsigned short addr, char *rxdata, int length)
{
    int ret = -1;
    int retries = 0;
    unsigned char tmp_buf[2];

    struct rt_i2c_msg msgs[] =
    {
        {
            .addr   = ft5x06_driver.address,
            .flags = RT_I2C_WR,
            .len    = 2,
            .buf    = tmp_buf,
        },
        {
            .addr   = ft5x06_driver.address,
            .flags  = RT_I2C_RD,
            .len    = length,
            .buf    = (unsigned char *)rxdata,
        },
    };

    tmp_buf[0] = (unsigned char)(addr >> 8);
    tmp_buf[1] = (unsigned char)(addr);

    while (retries < IIC_RETRY_NUM)
    {
        ret = rt_i2c_transfer(ft5x06_i2c_bus, msgs, 2);
        if (ret == 2)break;
        retries++;
    }

    if (retries >= IIC_RETRY_NUM)
    {
        LOG_E("%s i2c read error", __func__);
        ret = -1;
    }

    return ret;
}

static void ft5x06_isr_enable(rt_bool_t enable)
{
    rt_pin_irq_enable(BSP_TOUCH_INT_PIN, enable);
}

static void ft5x06_touch_isr(void *parameter)
{
    ft5x06_isr_enable(RT_FALSE);
    rt_sem_release(ft5x06_driver.isr_sem);
}

static rt_err_t ft5x06_read_point(touch_msg_t msg)
{
    int ret = -1;
    char buf[POINT_SIZE + 3];
    ponit_info_t point;

    memset(buf, 0, sizeof(buf));
    ret = ft5x06_read(0x1000, buf, POINT_SIZE + 2);
    if (ret < 0)
    {
        return -RT_ERROR;
    }

    point.u8ID       = buf[2];
    point.u16PosX    = (((uint16_t)buf[4]) << 8) + buf[3];
    point.u16PosY    = (((uint16_t)buf[6]) << 8) + buf[5];
    point.u8Pressure = buf[7];
    point.u8EventId  = buf[8];

    msg->x = point.u16PosX;
    msg->y = point.u16PosY;

    if (point.u8EventId == 1)
    {
        msg->event = TOUCH_EVENT_DOWN;
    }
    else if (point.u8EventId == 2)
    {
        msg->event = TOUCH_EVENT_MOVE;
    }
    else if (point.u8EventId == 4)
    {
        msg->event = TOUCH_EVENT_UP;
    }
    else
    {
        msg->event = TOUCH_EVENT_NONE;
        return -RT_ERROR;
    }
    
    dbg_log(DBG_LOG, "x:%d y:%d status:%s", msg->x, msg->y, 
        msg->event == TOUCH_EVENT_DOWN ? "DOWN" : (msg->event == TOUCH_EVENT_MOVE \
        ? "MOVE" : (msg->event == TOUCH_EVENT_UP ? "UP" : "NONE")));

    return RT_EOK;
}

static void ft5x06_init(struct rt_i2c_bus_device *i2c_bus)
{
    if (ft5x06_i2c_bus == RT_NULL)
    {
        ft5x06_i2c_bus = i2c_bus;
    }
    ft5x06_driver.isr_sem = rt_sem_create("ft5x06", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(ft5x06_driver.isr_sem);

    ft5x06_isr_enable(RT_FALSE);
    rt_pin_mode(BSP_TOUCH_INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(BSP_TOUCH_INT_PIN, PIN_IRQ_MODE_FALLING, ft5x06_touch_isr, RT_NULL);
    ft5x06_isr_enable(RT_TRUE);
	
    rt_thread_mdelay(200);
}

static void ft5x06_deinit(void)
{
    if (ft5x06_driver.isr_sem)
    {
        rt_sem_delete(ft5x06_driver.isr_sem);
    }
}

struct touch_ops ft5x06_ops =
{
    ft5x06_isr_enable,
    ft5x06_read_point,
    ft5x06_init,
    ft5x06_deinit,
};

static rt_bool_t ft5x06_probe(struct rt_i2c_bus_device *i2c_bus)
{
    int err = 0;
    char tmp[2];
    short CurVersion;

    ft5x06_i2c_bus = i2c_bus;
    rt_memset(tmp, 0, sizeof(tmp));
    err = ft5x06_read(0x000c, tmp, 2);
    if (err < 0) 
    {
        LOG_E("%s failed: %d", __func__, err);
        return RT_FALSE;
    }
    CurVersion = (tmp[0]<<8) | tmp[1];
    LOG_I("touch Version:%d", CurVersion);

    return RT_TRUE;
}

int ft5x06_driver_register(void)
{
    ft5x06_driver.address = FT5x06_TS_ADDR;//0x48?
    ft5x06_driver.probe = ft5x06_probe;
    ft5x06_driver.ops = &ft5x06_ops;
    ft5x06_driver.user_data = RT_NULL;
    rt_touch_drivers_register(&ft5x06_driver);
    return 0;
}

INIT_DEVICE_EXPORT(ft5x06_driver_register);

#endif
