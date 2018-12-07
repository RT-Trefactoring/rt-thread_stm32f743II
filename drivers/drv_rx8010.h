#ifndef __DRV_RX8010_H
#define __DRV_RX8010_H

#include <board.h>
#include <finsh.h>
#include <rtdevice.h>
#include <rthw.h>


extern int rx8010_get_time(void);

extern int rx8010_set_time(time_t time_stamp);

extern int rt_rx8010_init(void);
#endif

