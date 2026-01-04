#ifndef __EEPROM_H__
#define __EEPROM_H__

#ifdef __cplusplus
extern "C" {
#endif


void Save_Band_Eeprom(uint8_t actual_mem, uint8_t save_mem);

void Eeprom_setup(void);
void Eeprom_loop(void);











#ifdef __cplusplus
}
#endif
#endif
