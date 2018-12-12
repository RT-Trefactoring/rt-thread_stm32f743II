/*
 * File      : drv_ramdisk.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-03-19     Liuguang     the first version.
 */
#include <rtthread.h>

#define SECTOR_SIZE     (512)
#define DISK_SIZE       (BSP_USING_RAMDISK_SIZE * 1024 * 1024)

static struct rt_device ram_disk_device;
static rt_uint32_t *disk_addr;

static rt_size_t flash_disk_read(rt_device_t dev,
                                 rt_off_t pos,
                                 void *buffer,
                                 rt_size_t size)
{
    if((pos + size) * SECTOR_SIZE <= DISK_SIZE)
    {
        rt_memcpy(buffer, (rt_uint8_t *)disk_addr + SECTOR_SIZE * pos, SECTOR_SIZE * size);
        return size;
    }
    return 0;
}

static rt_size_t flash_disk_write(rt_device_t dev,
                                  rt_off_t pos,
                                  const void *buffer,
                                  rt_size_t size)
{
    if((pos + size) * SECTOR_SIZE <= DISK_SIZE)
    {
        rt_memcpy((rt_uint8_t *)disk_addr + SECTOR_SIZE * pos, buffer, SECTOR_SIZE * size);
        return size;
    }
    return 0;
}

static rt_err_t flash_disk_control(rt_device_t dev, int cmd, void *args)
{
    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry;

        geometry = (struct rt_device_blk_geometry *)args;
        if (geometry == RT_NULL) return -RT_ERROR;

        geometry->bytes_per_sector = SECTOR_SIZE;
        geometry->sector_count = DISK_SIZE / SECTOR_SIZE;
        geometry->block_size = SECTOR_SIZE;
    }
    return RT_EOK;
}

int rt_hw_ramdisk_init(void)
{
    disk_addr = rt_malloc(DISK_SIZE);
    if (disk_addr == RT_NULL)
    {
        rt_kprintf("%s init failed\n", __FUNCTION__);
        return -RT_ERROR;
    }

    rt_memset(&ram_disk_device, 0, sizeof(ram_disk_device));

    ram_disk_device.type    = RT_Device_Class_Block;
    ram_disk_device.init    = RT_NULL;
    ram_disk_device.open    = RT_NULL;
    ram_disk_device.close   = RT_NULL;
    ram_disk_device.read    = flash_disk_read;
    ram_disk_device.write   = flash_disk_write;
    ram_disk_device.control = flash_disk_control;

    /* register device */
    return rt_device_register(&ram_disk_device, "ram0", \
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}
INIT_DEVICE_EXPORT(rt_hw_ramdisk_init);
