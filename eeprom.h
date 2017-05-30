
#ifndef __EEPROM_H__
#define __EEPROM_H__

void SetAVS(unsigned char new_value); // Auto height control plasma arc voltage
void SetIHS(unsigned char new_value); // Initial positioning height for plasma torch
void SetTLS(unsigned char new_value); // Torch lift sensetivity for oxyfuel
unsigned char GetAVS();
unsigned char GetIHS();
unsigned char GetTLS();

#endif /*__EEPROM_H_*/
