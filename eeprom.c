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

//  EEPROM base address
#define EEPROM_BASE_ADDRESS 0x4000  

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

void SetAVS(unsigned char new_value){
  EEPROM_writeChar(0, new_value);
}

void SetIHS(unsigned char new_value){
  setInitialHeight(new_value);
  EEPROM_writeChar(1, new_value);
}

void SetTLS(unsigned char new_value){
  setSlowVelocity(new_value);
  EEPROM_writeChar(2, new_value);
};

unsigned char GetAVS(){
  return EEPROM_readChar(0);
}

unsigned char GetIHS(){
  char result = EEPROM_readChar(1);
  setInitialHeight(result);
  return result;
}

unsigned char GetTLS(){
  char result = EEPROM_readChar(2);
  setSlowVelocity(result);
  return result;
};
