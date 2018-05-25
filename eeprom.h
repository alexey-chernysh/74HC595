
#ifndef __EEPROM_H__
#define __EEPROM_H__

void StoreVoltageSettingInEEPROM(unsigned char new_value); // Auto height control plasma arc voltage
void StoreInitialHeightSettingInEEPROM(unsigned char new_value); // Initial positioning height for plasma torch
void StoreOxyfuelLiftSlowingSettingInEEPROM(unsigned char new_value); // Torch lift sensetivity for oxyfuel
void StoreLiftVelocitySettingInEEPROM(unsigned char new_value);
unsigned char RestoreVoltageSettingFromEEPROM();
unsigned char RestoreInitialHeightSettingFromEEPROM();
unsigned char RestoreOxyfuelLiftSlowingSettingFromEEPROM();
unsigned char RestoreLiftVelocitySettingIFromEEPROM();

#endif /*__EEPROM_H_*/
