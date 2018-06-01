/**
  ******************************************************************************
  * @file eeprom.c
  * @brief STM8S eeprom utilites, applied for store end recovery of arc voltage & initial height 
  * @author Alexey Chernysh
  * @version V0.2
  * @date 09-09-2016
  ******************************************************************************
  */

#include "common.h"
#include "adc.h"
#include "eeprom.h"
#include "motor_control.h"

//  EEPROM base address
#define EEPROM_BASE_ADDRESS 0x4000  
#define VOLTAGE_OFFSET 0x0
#define INITIAL_HEIGHT_OFFSET 0x1
#define OXIFUEL_LIFT_SLOWING_OFFSET 0x2
#define LIFT_VELOCITY_OFFSET 0x1

void EEPROM_writeChar(int offset, char data){
  if (FLASH_IAPSR_DUL == 0){ //  Проверить флаг EEPROM  - включена ли защита от записи  
    // Снять защиту
    FLASH_DUKR = 0xae;
    FLASH_DUKR = 0x56;
  }
  
  char *address = (char *) EEPROM_BASE_ADDRESS;
  address += offset;
  *address = data;  
  
  FLASH_IAPSR_DUL = 0; //  Восстановить защиту
}

char EEPROM_readChar(int offset){
  char *address = (char *) EEPROM_BASE_ADDRESS;
  address += offset;
  return *address;
}

void StoreVoltageSettingInEEPROM(unsigned char new_value){
  EEPROM_writeChar(VOLTAGE_OFFSET, new_value);
}

void StoreInitialHeightSettingInEEPROM(unsigned char new_value){
  EEPROM_writeChar(INITIAL_HEIGHT_OFFSET, new_value);
}

void StoreOxyfuelLiftSlowingSettingInEEPROM(unsigned char new_value){
  EEPROM_writeChar(OXIFUEL_LIFT_SLOWING_OFFSET, new_value);
};

void StoreLiftVelocitySettingInEEPROM(unsigned char new_value){
  EEPROM_writeChar(LIFT_VELOCITY_OFFSET, new_value);
  unsigned int pwm_limit = 127-new_value;
  SetPWMLimit(pwm_limit);
};

unsigned char RestoreVoltageSettingFromEEPROM(){
  return EEPROM_readChar(VOLTAGE_OFFSET);
}

unsigned char RestoreInitialHeightSettingFromEEPROM(){
  return EEPROM_readChar(INITIAL_HEIGHT_OFFSET);
}

unsigned char RestoreOxyfuelLiftSlowingSettingFromEEPROM(){
  return EEPROM_readChar(OXIFUEL_LIFT_SLOWING_OFFSET);
};

unsigned char RestoreLiftVelocitySettingFromEEPROM(){
  return EEPROM_readChar(LIFT_VELOCITY_OFFSET);
};
