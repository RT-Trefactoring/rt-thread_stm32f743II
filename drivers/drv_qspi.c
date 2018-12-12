#include <rtthread.h>
#include <board.h>
#include <string.h>
#include <rtdevice.h>
#include <drv_qspi.h>
#include "stm32_qspi.h"

void QUADSPI_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_QSPI_IRQHandler(&QSPIHandle);
    rt_interrupt_leave();
}

struct rt_mtd_nor_device qspi_disk_device;
static rt_size_t mtd_dev_read(struct rt_mtd_nor_device* device, rt_off_t offset, rt_uint8_t* data, rt_uint32_t length)
{
    RT_ASSERT(device != RT_NULL);

    if(BSP_QSPI_Read(data, offset, length) != QSPI_OK)
    {
        return 0;
    }

    return length;
}

static rt_size_t mtd_dev_write(struct rt_mtd_nor_device* device, rt_off_t offset, const rt_uint8_t* data, rt_uint32_t length)
{
    RT_ASSERT(device != RT_NULL);

    if (BSP_QSPI_Write((uint8_t *)data, offset, length) != QSPI_OK)
    {
        return 0;
    }

    return length;
}

static rt_err_t mtd_dev_erase(struct rt_mtd_nor_device* device, rt_off_t offset, rt_uint32_t length)
{
    uint8_t ret = 0;
    QSPI_Info info;
    
    RT_ASSERT(device != RT_NULL);
    
    BSP_QSPI_GetInfo(&info);
    
    while(length > 0)
    {
        if (BSP_QSPI_Erase_Block(offset) != QSPI_OK)
        {
            return -RT_ERROR;
        }
        length -= info.EraseSectorSize;
        offset += info.EraseSectorSize;
    }
    return RT_EOK;
}

static const struct rt_mtd_nor_driver_ops _ops = 
{
    RT_NULL,
    mtd_dev_read,
    mtd_dev_write,
    mtd_dev_erase,
};
static int rt_qspi_disk_init(void)
{
    QSPI_Info info;
    
    BSP_QSPI_Init();
    rt_memset(&qspi_disk_device, 0, sizeof(qspi_disk_device));

    BSP_QSPI_GetInfo(&info);
    qspi_disk_device.block_start = 0;
    qspi_disk_device.block_end = info.EraseSectorsNumber;
    qspi_disk_device.block_size = info.EraseSectorSize;
    qspi_disk_device.ops = &_ops;

    /* register device */
    return rt_mtd_nor_register_device("qmtd0", &qspi_disk_device);
}
INIT_DEVICE_EXPORT(rt_qspi_disk_init);
