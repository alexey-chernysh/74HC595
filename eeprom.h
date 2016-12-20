
#ifndef __EEPROM_H__
#define __EEPROM_H__

void SetAVS(unsigned char new_value);
void SetIHS(unsigned char new_value);
unsigned char GetAVS();
unsigned char GetIHS();

#endif /*__EEPROM_H_*/
