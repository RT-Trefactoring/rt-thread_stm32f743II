#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Configuration */

#define BOARD_STM32H743_MANGO

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_USING_IDLE_HOOK
#define RT_IDEL_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 256
#define RT_USING_TIMER_SOFT
#define RT_TIMER_THREAD_PRIO 4
#define RT_TIMER_THREAD_STACK_SIZE 512
#define RT_DEBUG

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_USING_MEMHEAP_AS_HEAP
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x40000
#define ARCH_ARM
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M7

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10

/* C++ features */


/* Command shell */

#define RT_USING_FINSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_CMD_SIZE 80
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
#define FINSH_ARG_MAX 10

/* Device virtual file system */

#define RT_USING_DFS
#define DFS_USING_WORKDIR
#define DFS_FILESYSTEMS_MAX 2
#define DFS_FILESYSTEM_TYPES_MAX 2
#define DFS_FD_MAX 4
#define RT_USING_DFS_ELMFAT

/* elm-chan's FatFs, Generic FAT Filesystem Module */

#define RT_DFS_ELM_CODE_PAGE 437
#define RT_DFS_ELM_WORD_ACCESS
#define RT_DFS_ELM_USE_LFN_0
#define RT_DFS_ELM_USE_LFN 0
#define RT_DFS_ELM_MAX_LFN 255
#define RT_DFS_ELM_DRIVES 2
#define RT_DFS_ELM_MAX_SECTOR_SIZE 512
#define RT_DFS_ELM_REENTRANT
#define RT_USING_DFS_DEVFS
#define RT_USING_DFS_ROMFS

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL
#define RT_USING_CPUTIME
#define RT_USING_CPUTIME_CORTEXM
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS
#define RT_USING_PIN
#define RT_USING_MTD_NOR

/* Using WiFi */


/* Using USB */

#define RT_USING_USB_DEVICE
#define RT_USBD_THREAD_STACK_SZ 4096
#define USB_VENDOR_ID 0x0483
#define USB_PRODUCT_ID 0x0987
#define _RT_USB_DEVICE_CDC
#define RT_USB_DEVICE_CDC
#define RT_VCOM_TASK_STK_SIZE 512
#define RT_VCOM_SERNO "32021919830108"
#define RT_VCOM_SER_LEN 14
#define RT_VCOM_TX_TIMEOUT 1000

/* POSIX layer and C standard library */

#define RT_USING_LIBC
#define RT_USING_POSIX

/* Network */

/* Socket abstraction layer */


/* light weight TCP/IP stack */


/* Modbus master and slave stack */


/* AT commands */


/* VBUS(Virtual Software BUS) */


/* Utilities */


/* ARM CMSIS */


/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */


/* IoT Cloud */


/* security packages */


/* language packages */


/* multimedia packages */


/* tools packages */


/* system packages */

#define PKG_USING_GUIENGINE
#define PKG_USING_GUIENGINE_LATEST_VERSION
#define GUIENGINE_NAME_MAX 16
#define GUIENG_USING_FNT_FILE
#define GUIENGINE_USING_FONT16
#define GUIENGINE_USING_FONT12
#define GUIENGINE_USING_PNG
#define GUIENGINE_IMAGE_LODEPNG
#define GUIENGINE_IMAGE_CONTAINER
#define GUIENGINE_USING_DEMO
#define PKG_USING_LITTLEFS
#define PKG_USING_LITTLEFS_V090
#define LFS_READ_SIZE 256
#define LFS_WRITE_SIZE 256
#define LFS_BLOCK_SIZE 4096
#define LFS_LOOKAHEAD 512

/* peripheral libraries and drivers */


/* miscellaneous packages */


/* samples: kernel and components samples */


/* Privated Packages of RealThread */


/* Network Utilities */


/* rtpkgs online packages */


/* Board Config */

/* Memory Config */

#define BSP_USING_SDRAM
#define BSP_SDRAM_SIZE 32

/* UART Config */

#define BSP_USING_UART1

/* USB Config */

#define BSP_USING_USBD

/* LCD Config */

#define BSP_USING_LCD
#define BSP_LCD_HEIGHT 480
#define BSP_LCD_WIDTH 800
#define BSP_LCD_HSYNC 1
#define BSP_LCD_HBP 46
#define BSP_LCD_HFP 210
#define BSP_LCD_VSYNC 1
#define BSP_LCD_VBP 23
#define BSP_LCD_VFP 22

/* I2C Config */

#define BSP_USING_I2C1
#define BSP_I2C1_NAME "i2c1"
#define BSP_I2C1_SDA 134
#define BSP_I2C1_SCL 83

/* CTouchPad Config */

#define BSP_USING_TOUCH
#define BSP_TOUCH_INT_PIN 84
#define BSP_TOUCH_RST_PIN 7

/* Select ramdisk drivers */


/* Select qspi flash drivers */

#define BSP_USING_PIN

#endif
