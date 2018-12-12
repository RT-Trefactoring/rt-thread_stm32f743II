#include <rtthread.h>
#include <stdio.h>

#if defined(RT_USING_DFS)
#include <dfs_file.h>
#include <dfs_posix.h>
#endif
int main(void)
{
	printf("Hello World!\r\n");
	
#if defined(BSP_USING_RAMDISK) && defined(BSP_USING_RAMDISK_MOUNT) 
    dfs_mkfs("elm", "ram0"); 
    
    if(dfs_mount("ram0", BSP_USING_RAMDISK_PATH_MOUNT, "elm", 0, 0) != 0)
    {
        rt_kprintf("sdcard mount '%s' failed.\n", BSP_USING_RAMDISK_PATH_MOUNT);  
    }
#endif

#ifdef BSP_USING_QSPI_FLASH_MOUNT
    if(dfs_mount("qmtd0", BSP_USING_QSPI_FLASH_PATH_MOUNT, "lfs", 0, 0) != 0)
    {
        dfs_mkfs("lfs", "qmtd0"); 
        if(dfs_mount("qmtd0", BSP_USING_QSPI_FLASH_PATH_MOUNT, "lfs", 0, 0) != 0)
        {
            rt_kprintf("qmtd0 mount '%s' failed.\n", BSP_USING_QSPI_FLASH_PATH_MOUNT); 
        }
    }
#endif

	return 0;
}
