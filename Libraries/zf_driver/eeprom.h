#ifndef _EEPROM_H_
#define _EEPROM_H_
#include "zf_common_headfile.h"
void eeprom_init(void);
void eeprom_write_value(uint32 addr, uint16 max, uint16 min);
void eeprom_read_value(uint32 addr, uint16 *buff);
void eeprom_read(uint32 addr, uint8 *buf, uint16 len);

#endif
