#include "drv_sdio_sd.h"
#include "string.h"
#include <rtthread.h>
#include <dfs_fs.h>


SD_HandleTypeDef        SDCARD_Handler;     
HAL_SD_CardInfoTypeDef  SDCardInfo;         //SD����Ϣ�ṹ��
DMA_HandleTypeDef SDTxDMAHandler,SDRxDMAHandler;    //SD��DMA���ͺͽ��վ��


//SD����ʼ��
//����ֵ:0 ��ʼ����ȷ������ֵ����ʼ������
rt_uint8_t SD_Init(void)
{
    rt_uint8_t SD_Error;
    
    //��ʼ��ʱ��ʱ�Ӳ��ܴ���400KHZ 
    SDCARD_Handler.Instance=SDMMC1;
    SDCARD_Handler.Init.ClockEdge=SDMMC_CLOCK_EDGE_RISING;          //������     
    SDCARD_Handler.Init.ClockPowerSave=SDMMC_CLOCK_POWER_SAVE_DISABLE;    //����ʱ���ر�ʱ�ӵ�Դ
    SDCARD_Handler.Init.BusWide=SDMMC_BUS_WIDE_4B;                        //1λ������
    SDCARD_Handler.Init.HardwareFlowControl=SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;//�ر�Ӳ������
    SDCARD_Handler.Init.ClockDiv=SDMMC_NSpeed_CLK_DIV;                 //��ʼ��ʱ��Ϊ400KHZ
    
    SD_Error=HAL_SD_Init(&SDCARD_Handler);
    if(SD_Error!=HAL_OK) return 1;
    
  //��ȡSD����Ϣ
	HAL_SD_GetCardInfo(&SDCARD_Handler,&SDCardInfo);
    return 0;
}

//SDMMC�ײ�������ʱ��ʹ�ܣ��������ã�DMA����
//�˺����ᱻHAL_SD_Init()����
//hsd:SD�����
void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    DMA_HandleTypeDef TxDMAHandler,RxDMAHandler;
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_SDMMC1_CLK_ENABLE();  //ʹ��SDMMC1ʱ��
    __HAL_RCC_GPIOC_CLK_ENABLE();   //ʹ��GPIOCʱ��
    __HAL_RCC_GPIOD_CLK_ENABLE();   //ʹ��GPIODʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //���츴��
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //����
    GPIO_Initure.Alternate=GPIO_AF12_SDIO1; //����ΪSDIO
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);     //��ʼ��
    
    //PD2
    GPIO_Initure.Pin=GPIO_PIN_2;            
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);     //��ʼ��


#if (SD_DMA_MODE==1)                        //ʹ��DMAģʽ
    HAL_NVIC_SetPriority(SDMMC1_IRQn,2,0);  //����SDMMC1�жϣ���ռ���ȼ�2�������ȼ�0
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);        //ʹ��SDMMC1�ж�
    
    //���÷���DMA
    SDRxDMAHandler.Instance=DMA2_Stream3;
    SDRxDMAHandler.Init.Channel=DMA_CHANNEL_4;
    SDRxDMAHandler.Init.Direction=DMA_PERIPH_TO_MEMORY;
    SDRxDMAHandler.Init.PeriphInc=DMA_PINC_DISABLE;
    SDRxDMAHandler.Init.MemInc=DMA_MINC_ENABLE;
    SDRxDMAHandler.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
    SDRxDMAHandler.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;
    SDRxDMAHandler.Init.Mode=DMA_PFCTRL;
    SDRxDMAHandler.Init.Priority=DMA_PRIORITY_VERY_HIGH;
    SDRxDMAHandler.Init.FIFOMode=DMA_FIFOMODE_ENABLE;
    SDRxDMAHandler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;
    SDRxDMAHandler.Init.MemBurst=DMA_MBURST_INC4;
    SDRxDMAHandler.Init.PeriphBurst=DMA_PBURST_INC4;

    __HAL_LINKDMA(hsd, hdmarx, SDRxDMAHandler); //������DMA��SD���ķ���DMA��������
    HAL_DMA_DeInit(&SDRxDMAHandler);
    HAL_DMA_Init(&SDRxDMAHandler);              //��ʼ������DMA
    
    //���ý���DMA 
    SDTxDMAHandler.Instance=DMA2_Stream6;
    SDTxDMAHandler.Init.Channel=DMA_CHANNEL_4;
    SDTxDMAHandler.Init.Direction=DMA_MEMORY_TO_PERIPH;
    SDTxDMAHandler.Init.PeriphInc=DMA_PINC_DISABLE;
    SDTxDMAHandler.Init.MemInc=DMA_MINC_ENABLE;
    SDTxDMAHandler.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
    SDTxDMAHandler.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;
    SDTxDMAHandler.Init.Mode=DMA_PFCTRL;
    SDTxDMAHandler.Init.Priority=DMA_PRIORITY_VERY_HIGH;
    SDTxDMAHandler.Init.FIFOMode=DMA_FIFOMODE_ENABLE;
    SDTxDMAHandler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;
    SDTxDMAHandler.Init.MemBurst=DMA_MBURST_INC4;
    SDTxDMAHandler.Init.PeriphBurst=DMA_PBURST_INC4;
    
    __HAL_LINKDMA(hsd, hdmatx, SDTxDMAHandler);//������DMA��SD���ķ���DMA��������
    HAL_DMA_DeInit(&SDTxDMAHandler);
    HAL_DMA_Init(&SDTxDMAHandler);              //��ʼ������DMA 
  

    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 3, 0);  //����DMA�ж����ȼ�
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 3, 0);  //����DMA�ж����ȼ�
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
#endif
}

//�õ�����Ϣ
//cardinfo:����Ϣ�洢��
//����ֵ:����״̬
rt_uint8_t SD_GetCardInfo(HAL_SD_CardInfoTypeDef *cardinfo)
{
    rt_uint8_t sta;
	sta=HAL_SD_GetCardInfo(&SDCARD_Handler,cardinfo);
    return sta;
}
 #if (SD_DMA_MODE==1)        //DMAģʽ

//ͨ��DMA��ȡSD��һ������
//buf:�����ݻ�����
//sector:������ַ
//blocksize:������С(һ�㶼��512�ֽ�)
//cnt:��������	
//����ֵ:����״̬;0,����;����,�������;
rt_uint8_t SD_ReadBlocks_DMA(uint32_t *buf,uint64_t sector ,uint32_t cnt)
{
    rt_uint8_t err=SD_OK;
    err=HAL_SD_ReadBlocks_DMA(&SDCARD_Handler,buf,sector,SECTOR_SIZE,cnt);//ͨ��DMA��ȡSD��һ������
    if(err==SD_OK)//��ȡ�ɹ�
    {
        //�ȴ���ȡ���
        err=HAL_SD_CheckReadOperation(&SDCARD_Handler,(uint32_t)SD_TIMEOUT);
    }

    return err;
}

//дSD��
//buf:д���ݻ�����
//sector:������ַ
//blocksize:������С(һ�㶼��512�ֽ�)
//cnt:��������	
//����ֵ:����״̬;0,����;����,�������;	
rt_uint8_t SD_WriteBlocks_DMA(uint32_t *buf,uint64_t sector,uint32_t cnt)
{
    rt_uint8_t err=SD_OK; 
    err=HAL_SD_WriteBlocks_DMA(&SDCARD_Handler,buf,sector,SECTOR_SIZE,cnt);//ͨ��DMAдSD��һ������
    if(err==SD_OK)//д�ɹ�
    {     
       err=HAL_SD_CheckWriteOperation(&SDCARD_Handler,(uint32_t)SD_TIMEOUT);//�ȴ���ȡ���/
    }
    return err;
}


//SDMMC1�жϷ�����
void SDMMC1_IRQHandler(void)
{
    HAL_SD_IRQHandler(&SDCARD_Handler);
}

//DMA2������6�жϷ�����
void DMA2_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(SDCARD_Handler.hdmatx);
}

//DMA2������3�жϷ�����
void DMA2_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(SDCARD_Handler.hdmarx);
}
#else                                   //��ѵģʽ
 
//��ȡSD������
//buf:�����ݻ�����
//sector:������ַ
//cnt:��������	
//����ֵ:����״̬;0,����;����,�������;
rt_uint8_t SD_ReadBlocks(uint32_t *buf,uint64_t sector ,uint32_t cnt)
{
    rt_uint8_t err=HAL_OK;
	rt_uint32_t timeout=SD_TIMEOUT; 
    err=HAL_SD_ReadBlocks(&SDCARD_Handler,(rt_uint8_t*)buf,sector,cnt,timeout);//ͨ��DMA��ȡSD��һ������
    return err;
}

//дSD����
//buf:д���ݻ�����
//sector:������ַ
//cnt:��������	
//����ֵ:����״̬;0,����;����,�������;	
rt_uint8_t SD_WriteBlocks(uint32_t *buf,uint64_t sector,uint32_t cnt)
{
	rt_uint32_t timeout =SD_TIMEOUT;
    return HAL_SD_WriteBlocks(&SDCARD_Handler,(rt_uint8_t*)buf,sector,cnt,timeout);
}
#endif

/*
 * RT-Thread SD Card Driver
 * 20100715 Bernard support SDHC card great than 4G.
 * 20110905 JoyChen support to STM32F2xx
 */




static struct rt_device sdcard_device;
static struct rt_semaphore sd_lock;

/* RT-Thread Device Driver Interface */
static rt_err_t rt_sdcard_init(rt_device_t dev)
{
	if (rt_sem_init(&sd_lock, "sdlock", 1, RT_IPC_FLAG_FIFO) != RT_EOK)
	{
		rt_kprintf("init sd lock semaphore failed\n");
	}
	else
		rt_kprintf("SD Card init OK\n");

	return RT_EOK;
}

static rt_err_t rt_sdcard_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t rt_sdcard_close(rt_device_t dev)
{
	return RT_EOK;
}

static uint32_t sdio_buffer[512/sizeof(uint32_t)];
static rt_size_t rt_sdcard_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	rt_uint8_t status = HAL_OK;

	rt_sem_take(&sd_lock, RT_WAITING_FOREVER);

    if(((uint32_t)buffer & 0x03) != 0)
    {
        /* non-aligned. */
        uint32_t i;
        uint64_t sector_adr;
        uint8_t* copy_buffer;

        sector_adr = (uint64_t)pos*SECTOR_SIZE;
        copy_buffer = (uint8_t*)buffer;

        for(i=0; i<size; i++)
        {
            #if (SD_DMA_MODE==1)   
            status=SD_ReadBlocks_DMA(sdio_buffer,sector_adr,1);//ͨ��DMAдSD��һ������
            #else
            status=SD_ReadBlocks(sdio_buffer,sector_adr,1);
            #endif
            memcpy(copy_buffer, sdio_buffer, SECTOR_SIZE);
            sector_adr += SECTOR_SIZE;
            copy_buffer += SECTOR_SIZE;
        }
    }
    else
    {
        #if (SD_DMA_MODE==1) 
        status=SD_ReadBlocks_DMA(buffer,(uint64_t)pos*SECTOR_SIZE,size);//ͨ��DMAдSD��һ������
        #else
        SD_ReadBlocks(buffer,(uint64_t)pos*SECTOR_SIZE,size);
        #endif
    }


	rt_sem_release(&sd_lock);
	if (status == HAL_OK) return size;

	rt_kprintf("read failed: %d, buffer 0x%08x\n", status, buffer);
	return 0;
}

static rt_size_t rt_sdcard_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	rt_uint8_t status = HAL_OK;
	rt_sem_take(&sd_lock, RT_WAITING_FOREVER);

    if(((uint32_t)buffer & 0x03) != 0)
    {
        /* non-aligned. */
        uint32_t i;
        uint64_t sector_adr;
        uint8_t* copy_buffer;

        sector_adr = (uint64_t)pos*SECTOR_SIZE;
        copy_buffer = (uint8_t*)buffer;

        for(i=0; i<size; i++)
        {
            memcpy(sdio_buffer, copy_buffer, SECTOR_SIZE);
            #if (SD_DMA_MODE==1)    
            status=SD_WriteBlocks_DMA((uint32_t*)sdio_buffer,sector_adr,1);
            #else
            status=SD_WriteBlocks((uint32_t*)sdio_buffer,sector_adr,1);
            #endif
            sector_adr += SECTOR_SIZE;
            copy_buffer += SECTOR_SIZE;
            
        }
    }
    else
    {
        #if (SD_DMA_MODE==1)   
        status=SD_WriteBlocks_DMA((uint32_t*)buffer,(uint64_t)pos*SECTOR_SIZE,size);
        #else
        status=SD_WriteBlocks((uint32_t*)buffer,(uint64_t)pos*SECTOR_SIZE,size);
        #endif
    }
	// }
	rt_sem_release(&sd_lock);
	if (status == HAL_OK) return size;
	rt_kprintf("write failed: %d, buffer 0x%08x\n", status, buffer);
	return 0;
}

static rt_err_t rt_sdcard_control(rt_device_t dev, int cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry;

        geometry = (struct rt_device_blk_geometry *)args;
        if (geometry == RT_NULL) return -RT_ERROR;

        geometry->bytes_per_sector = 512;
        geometry->block_size = SDCardInfo.BlockSize;
		geometry->sector_count = SDCardInfo.BlockNbr/SDCardInfo.BlockSize;
    }

	return RT_EOK;
}

int rt_hw_sdcard_init(void)
{
	if (SD_Init() == HAL_OK)
	{
		/* register sdcard device */
		sdcard_device.type  = RT_Device_Class_Block;
		sdcard_device.init 	= rt_sdcard_init;
		sdcard_device.open 	= rt_sdcard_open;
		sdcard_device.close = rt_sdcard_close;
		sdcard_device.read 	= rt_sdcard_read;
		sdcard_device.write = rt_sdcard_write;
		sdcard_device.control = rt_sdcard_control;

		/* no private */
		sdcard_device.user_data = &SDCardInfo;

		rt_device_register(&sdcard_device, "sd0",
			RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);

		return RT_EOK;
	}
	rt_kprintf("sdcard init failed\n");
    return 0;
}

INIT_DEVICE_EXPORT(rt_hw_sdcard_init);

