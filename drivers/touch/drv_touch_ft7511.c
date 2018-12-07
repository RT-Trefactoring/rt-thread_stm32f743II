/*
 * File      : drv_ft7511.c
 *             ft7511 touch driver
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
#include <rthw.h>
#include <rtdevice.h>
#include "drv_touch.h"
#include <string.h>

#ifdef BSP_USING_TOUCH

#define DBG_ENABLE
#define DBG_SECTION_NAME  "TOUCH.ft7511"
#define DBG_LEVEL         TOUCH_DBG_LEVEL
#define DBG_COLOR
#include <rtdbg.h>

static struct rt_i2c_bus_device *ft7511_i2c_bus;
static struct touch_drivers ft7511_driver;

static int ft7511_read(struct rt_i2c_bus_device *i2c_bus, rt_uint8_t addr, rt_uint8_t *buffer, rt_size_t length)
{
    int ret = -1;
    int retries = 0;

    struct rt_i2c_msg msgs[] =
    {
        {
            .addr   = ft7511_driver.address,
            .flags  = RT_I2C_WR,
            .len    = 1,
            .buf    = &addr,
        },
        {
            .addr   = ft7511_driver.address,
            .flags  = RT_I2C_RD,
            .len    = length,
            .buf    = buffer,
        },
    };

    while (retries < IIC_RETRY_NUM)
    {
        ret = rt_i2c_transfer(i2c_bus, msgs, 2);
        if (ret == 2)break;
        retries++;
    }

    if (retries >= IIC_RETRY_NUM)
    {
        LOG_E("%s i2c read error: %d", __func__, ret);
        return -1;
    }

    return ret;
}

static void ft7511_write(touch_drv_t driver, struct rt_i2c_bus_device *i2c_bus, rt_uint8_t addr, rt_uint8_t *buffer, rt_size_t length)
{

    rt_uint8_t *send_buffer = rt_malloc(length + 1);

    RT_ASSERT(send_buffer);

    send_buffer[0] = addr;
    memcpy(send_buffer + 1, buffer, length);

    struct rt_i2c_msg msgs[] =
    {
        {
            .addr   = ft7511_driver.address,
            .flags  = RT_I2C_WR,
            .len    = length + 1,
            .buf    = send_buffer,
        }
    };

    length = rt_i2c_transfer(i2c_bus, msgs, 1);
    rt_free(send_buffer);
    send_buffer = RT_NULL;
}

static void ft7511_isr_enable(rt_bool_t enable)
{
    rt_pin_irq_enable(BSP_TOUCH_INT_PIN, enable);
}

static void ft7511_touch_isr(void *parameter)
{
    ft7511_isr_enable(RT_FALSE);
    rt_sem_release(ft7511_driver.isr_sem);
}

static rt_err_t ft7511_read_point(touch_msg_t msg)
{
    int ret = -1;
    uint8_t point_num = 0;
    static uint8_t s_tp_down = 0;
    uint8_t point[6];
    ret = ft7511_read(ft7511_i2c_bus, 0x02, &point_num, 1);
    if (ret < 0)
    {
        return -RT_ERROR;
    }
    
    if (point_num == 0)
    {
        if (s_tp_down)
        {
            s_tp_down = 0;
            msg->event = TOUCH_EVENT_UP;
            return RT_EOK;
        }
        msg->event = TOUCH_EVENT_NONE;
        return RT_EOK;
    }
    
    ret = ft7511_read(ft7511_i2c_bus, 0x03, point, 6);
    if (ret < 0)
    {
        return -RT_ERROR;
    }

    msg->x = (point[0]&0x0F) << 8 | point[1];
    msg->y = (point[2]&0x0F) << 8 | point[3];
    if (s_tp_down)
    {
        msg->event = TOUCH_EVENT_MOVE;
        return RT_EOK;
    }
    msg->event = TOUCH_EVENT_DOWN;
    s_tp_down = 1;
    
    return RT_EOK;
}

static void ft7511_init(struct rt_i2c_bus_device *i2c_bus)
{
    if (ft7511_i2c_bus == RT_NULL)
    {
        ft7511_i2c_bus = i2c_bus;
    }
    ft7511_driver.isr_sem = rt_sem_create("ft7511", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(ft7511_driver.isr_sem);

    rt_pin_mode(BSP_TOUCH_INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(BSP_TOUCH_INT_PIN, PIN_IRQ_MODE_FALLING, ft7511_touch_isr, RT_NULL);

    rt_thread_mdelay(200);
}

static void ft7511_deinit(void)
{
    if (ft7511_driver.isr_sem)
    {
        rt_sem_delete(ft7511_driver.isr_sem);
        ft7511_driver.isr_sem = RT_NULL;
    }
}

struct touch_ops ft7511_ops =
{
    ft7511_isr_enable,
    ft7511_read_point,
    ft7511_init,
    ft7511_deinit,
};

static rt_bool_t ft7511_probe(struct rt_i2c_bus_device *i2c_bus)
{
    int err = 0;
    uint8_t cid = 0xFF;

    ft7511_i2c_bus = i2c_bus;
    err = ft7511_read(ft7511_i2c_bus, 0xA3, (uint8_t *)&cid, 1);
    if (err < 0)
    {
        LOG_E("%s failed: %d", __func__, err);
        return RT_FALSE;
    }
    LOG_I("touch CID:%02X", cid);
    if(cid == 0x54)
    {
        return RT_TRUE;
    }
    return RT_FALSE;
}

int ft7511_driver_register(void)
{
    ft7511_driver.address = 0x38;
    ft7511_driver.probe = ft7511_probe;
    ft7511_driver.ops = &ft7511_ops;
    ft7511_driver.user_data = RT_NULL;
    rt_touch_drivers_register(&ft7511_driver);
    return 0;
}

INIT_DEVICE_EXPORT(ft7511_driver_register);

#endif
