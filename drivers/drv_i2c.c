/*
 * File      : drv_i2c.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
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
 * 2017-06-05     tanek        first implementation.
 * 2018-03-08     ZYH          Porting for stm32f4xx
 */
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_i2c.h"
#include <board.h>
#include <string.h>

#define MAX_I2C_NUM     (8UL)

struct _drv_i2c_pin
{
    rt_base_t sda_pin;
    rt_base_t scl_pin;
};

#define GET_SCL_PIN(obj)  (((struct _drv_i2c_pin *)obj)->scl_pin)
#define GET_SDA_PIN(obj)  (((struct _drv_i2c_pin *)obj)->sda_pin)

static struct _drv_i2c_pin _drv_i2c_pin_pool[MAX_I2C_NUM];
static struct rt_i2c_bit_ops drv_bit_ops_pool[MAX_I2C_NUM];
static struct rt_i2c_bus_device i2c2_bus[MAX_I2C_NUM];

static void drv_set_sda(void *data, rt_int32_t state);
static void drv_set_scl(void *data, rt_int32_t state);
static rt_int32_t  drv_get_sda(void *data);
static rt_int32_t  drv_get_scl(void *data);
static void drv_udelay(rt_uint32_t us);
static const struct rt_i2c_bit_ops default_bit_ops =
{
    RT_NULL,
    drv_set_sda,
    drv_set_scl,
    drv_get_sda,
    drv_get_scl,
    drv_udelay,
    1,
    0
};


static struct rt_i2c_bit_ops * drv_i2c_create(int index, rt_base_t sda_pin, rt_base_t scl_pin)
{
    drv_bit_ops_pool[index] = default_bit_ops;
    drv_bit_ops_pool[index].data = &_drv_i2c_pin_pool[index];
    _drv_i2c_pin_pool[index].sda_pin = sda_pin;
    _drv_i2c_pin_pool[index].scl_pin = scl_pin;
    rt_pin_mode(sda_pin, PIN_MODE_OUTPUT_OD);
    rt_pin_mode(scl_pin, PIN_MODE_OUTPUT_OD);
    return &drv_bit_ops_pool[index];
}

static void drv_set_sda(void *data, rt_int32_t state)
{
    rt_pin_write(GET_SDA_PIN(data), state);
}

static void drv_set_scl(void *data, rt_int32_t state)
{
    rt_pin_write(GET_SCL_PIN(data), state);
}

static rt_int32_t  drv_get_sda(void *data)
{
    return rt_pin_read(GET_SDA_PIN(data));
}

static rt_int32_t  drv_get_scl(void *data)
{
    return rt_pin_read(GET_SCL_PIN(data));
}

static void drv_udelay(rt_uint32_t us)
{
    __IO uint32_t Delay = us * (SystemCoreClock / 8U / 1000000U);
    do 
    {
        __NOP();
    } 
    while (Delay --);
}

static int drv_register_i2c_bus(int index, const char * name, rt_base_t sda_pin, rt_base_t scl_pin)
{
    RT_ASSERT(index < MAX_I2C_NUM);
    memset((void *)&i2c2_bus[index], 0, sizeof(struct rt_i2c_bus_device));
    i2c2_bus[index].priv = drv_i2c_create(index, sda_pin, scl_pin);
    return rt_i2c_bit_add_bus(&i2c2_bus[index], name);
}

int drv_i2c_init(void)
{
#ifdef BSP_USING_I2C1
    drv_register_i2c_bus(0, BSP_I2C1_NAME, BSP_I2C1_SDA, BSP_I2C1_SCL);
#endif
#ifdef BSP_USING_I2C2
    drv_register_i2c_bus(1, BSP_I2C2_NAME, BSP_I2C2_SDA, BSP_I2C2_SCL);
#endif
#ifdef BSP_USING_I2C3
    drv_register_i2c_bus(2, BSP_I2C3_NAME, BSP_I2C3_SDA, BSP_I2C3_SCL);
#endif
#ifdef BSP_USING_I2C4
    drv_register_i2c_bus(3, BSP_I2C4_NAME, BSP_I2C4_SDA, BSP_I2C4_SCL);
#endif
#ifdef BSP_USING_I2C5
    drv_register_i2c_bus(4, BSP_I2C5_NAME, BSP_I2C5_SDA, BSP_I2C5_SCL);
#endif
#ifdef BSP_USING_I2C6
    drv_register_i2c_bus(5, BSP_I2C6_NAME, BSP_I2C6_SDA, BSP_I2C6_SCL);
#endif
#ifdef BSP_USING_I2C7
    drv_register_i2c_bus(6, BSP_I2C7_NAME, BSP_I2C7_SDA, BSP_I2C7_SCL);
#endif
#ifdef BSP_USING_I2C8
    drv_register_i2c_bus(7, BSP_I2C8_NAME, BSP_I2C8_SDA, BSP_I2C8_SCL);
#endif
    return RT_EOK;
}
INIT_DEVICE_EXPORT(drv_i2c_init);
