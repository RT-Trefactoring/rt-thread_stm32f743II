#ifndef __DRV_QSPI__
#define __DRV_QSPI__

extern int qspi_write(uint32_t address, uint8_t *buffer, uint32_t size);
extern int qspi_erase(uint32_t erase_start_address, uint32_t len);

#endif
