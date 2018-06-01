/**
  ******************************************************************************
  * @file serialLED.c
  * @brief STM8S clock initialization 
  * @author Alexey Chernysh
  * @version V0.2
  * @date 09-09-2016
  ******************************************************************************
  */

#include "common.h"
#include "encoder.h"
#include "adc.h"
#include "eeprom.h"
    
/*
    led bit map
      0
     ---
    |   |
   5|   |1
    | 6 |
     ---
    |   |
   4|   |2
    | 3 |
     --- |
         |7
    
    0 - visible
    1 - invisible
*/

static unsigned char int_to_7leds[] = {
  MAKE_BINARY(0,0,0,0,0,0,1,1), // 0
  MAKE_BINARY(1,0,0,1,1,1,1,1), // 1
  MAKE_BINARY(0,0,1,0,0,1,0,1), // 2
  MAKE_BINARY(0,0,0,0,1,1,0,1), // 3
  MAKE_BINARY(1,0,0,1,1,0,0,1), // 4
  MAKE_BINARY(0,1,0,0,1,0,0,1), // 5
  MAKE_BINARY(0,1,0,0,0,0,0,1), // 6
  MAKE_BINARY(0,0,0,1,1,1,1,1), // 7
  MAKE_BINARY(0,0,0,0,0,0,0,1), // 8
  MAKE_BINARY(0,0,0,0,1,0,0,1), // 9
  MAKE_BINARY(1,1,1,1,1,1,0,1), // -
  MAKE_BINARY(1,1,1,1,1,1,1,1), // space
  MAKE_BINARY(0,0,0,1,0,0,0,1), // A
  MAKE_BINARY(1,0,0,1,0,0,0,1), // H
  MAKE_BINARY(0,1,0,0,1,0,0,1), // S
  MAKE_BINARY(0,1,1,1,0,0,0,1), // F  
  MAKE_BINARY(0,1,1,0,0,0,0,1)  // E  
};

// MR/����� �������� � VCC, 
// QE ������� �� �����, 
// DSO ������� ���� ������� � DSI �������
// ��� ����������� ����� �������
#define SHK PB_ODR_bit.ODR0  // SHCP ClockPin - ������������
                             // �� ����� ������� ��� 3
                             // �� 74HC595  ��� 11
#define LCK PB_ODR_bit.ODR1  // STCP LatchPin - �������
                             // �� ����� ������� ��� 4
                             // �� 74HC595  ��� 12
#define DS  PB_ODR_bit.ODR2  // DSI  DataPin  - ������
                             // �� ����� ������� ��� 5
                             // �� 74HC595  ��� 14

#define BYTE_LENGTH 8

void SetDisplayState(char n,  // ����� ������� �� 0 �� 3
                     char mask){ // ������� ����� �������
  LCK = 0;  // �������� ����� - ������� �������
  int i;
  char tmp;
  char char2out = mask;
  for(i=0; i<BYTE_LENGTH; i++){
    tmp = char2out&1;
    DS = tmp;
    SHK = 0; 
    SHK = 1; //��������� ����� ������ ��������, �� �������� "����������" ������
    char2out = char2out>>1; // �������� �������� ���� �� ������� ������
  }
  char pos2out = (MAKE_BINARY(1,0,0,0,0,0,0,0))>>n;
  for(i=0; i<BYTE_LENGTH; i++){
    tmp = pos2out&1;
    DS = tmp;
    SHK = 0; 
    SHK = 1; //��������� ����� ������ ��������, �� �������� "����������" ������
    pos2out = pos2out>>1; // �������� �������� ���� �� ������� ������
  }
  LCK = 1;  // ����������� ����� - ������������� �������, ��� ����� ������������ �������� �� �������
}

/**
  ******************************************************************************
  * @brief SetupSerialLED function.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */

void SetupSerialLEDs(){

  PB_DDR_bit.DDR0 = 1;   // ����� PB0 ��������������� �� ����� �� ���� �������������� ���������� ��������
  PB_CR1_bit.C10 = 1;    // ����� ���� Push-pull
  PB_CR2_bit.C20 = 1;    // �������� ������������ - �� 10 ���.
  SHK = 0;
  
  PB_DDR_bit.DDR1 = 1;   // ����� PB1 ��������������� �� ����� �� ���� ������� ���������� ��������
  PB_CR1_bit.C11 = 1;    // ����� ���� Push-pull
  PB_CR2_bit.C21 = 1;    // �������� ������������ - �� 10 ���.
  LCK = 0;
  
  PB_DDR_bit.DDR2 = 1;   // ����� PB2 ��������������� �� ����� �� ���� ������ ���������� ��������
  PB_CR1_bit.C12 = 1;    // ����� ���� Push-pull
  PB_CR2_bit.C22 = 1;    // �������� ������������ - �� 10 ���.
  DS  = 0;
  
  // �� ������� 3 ����������� ����������� �� 7-���������� �����������
  TIM3_PSCR = 0x00;       //  Prescaler = 1.
  TIM3_ARRH = 0xFF;       //  High byte 
  TIM3_ARRL = 0xFF;       //  Low byte 
  TIM3_IER_UIE = 1;       //  Enable the update interrupts.
  TIM3_CR1_CEN = 1;       //  Finally enable the timer.
};

static unsigned char pos_counter = 0;
#define MAX_POS 3

static unsigned char value_refresh_counter = 0;
#define VALUE_REFRESH_COUNTER_LIMIT 4

static unsigned int param_display_counter = 0;
#define PARAM_DISPLAY_COUNTER_LIMIT 100

unsigned char ch0 = 8; 
unsigned char ch1 = 9; 
unsigned char ch2 = 10; 
unsigned char ch3 = 11; 

void SetDisplayNumeric(unsigned char lead_char, unsigned char num);

static unsigned char last_value2display = 0; 
static unsigned char new_value2display = 0; 

void ResetParamDisplayCounter(){
  param_display_counter = 0;
}

void displayValues(){
  if(preheatOn()) 
    SetValue2Display(3); 
  new_value2display = GetValue2Display();
  if(preheatOn()) 
    SetValue2Display(3); 
  switch(new_value2display){
  case 0:
    SetDisplayNumeric(11, GetCurrentVoltage());
    break;
  case 1:
    SetDisplayNumeric(12, RestoreVoltageSettingFromEEPROM());
    break;
  case 2:
    SetDisplayNumeric(13, RestoreInitialHeightSettingFromEEPROM());
    break;
  case 3:
    SetDisplayNumeric(14, RestoreOxyfuelLiftSlowingSettingFromEEPROM());
    break;
  case 4:
    SetDisplayNumeric(15, RestoreLiftVelocitySettingFromEEPROM());
    break;
  default:
    break;
  };
  if(last_value2display == new_value2display){
    param_display_counter++;
    if(param_display_counter > PARAM_DISPLAY_COUNTER_LIMIT){
      param_display_counter = 0;
      SetValue2Display(0);
    };
  } else {
    param_display_counter = 0;
    last_value2display = new_value2display;
  };
}

// ������ ���������� �� ���������� ��� ������������ �������3
#pragma vector = TIM3_OVR_UIF_vector
__interrupt void TIM3_OVR_UIF_handler(void)
{
  // ��������, ��� �� ������� ����������
  if (TIM3_SR1_UIF==1)
  {
    TIM3_SR1_UIF = 0; // ������� ����� ���������� �� ����������

    PB_ODR = 0;
    switch(pos_counter){
      case 0: 
        SetDisplayState(pos_counter, int_to_7leds[ch0]);
        break;
      case 1: 
        SetDisplayState(pos_counter, int_to_7leds[ch1]);
        break;
      case 2: 
        SetDisplayState(pos_counter, int_to_7leds[ch2]);
        break;
      case 3: 
        SetDisplayState(pos_counter, int_to_7leds[ch3]);
        break;
    }
    pos_counter++;
    if(pos_counter > MAX_POS) {
      pos_counter = 0;
      value_refresh_counter++;
    };
    if(value_refresh_counter > VALUE_REFRESH_COUNTER_LIMIT){
      value_refresh_counter = 0;
      displayValues();
    };
  }  
}

void SetDisplay(unsigned char c0, unsigned char c1, unsigned char c2, unsigned char c3){
  ch0 = c0;
  ch1 = c1;
  ch2 = c2;
  ch3 = c3;
}

void SetDisplayNumeric(unsigned char lead_char, unsigned char num){
  ch0 = lead_char;
  ch3 = num % 10;
  int tmp = num / 10;
  if(tmp > 0) {
    ch2 = tmp % 10;
    tmp = tmp / 10;
    if(tmp > 0) ch1 = tmp;
    else ch1 = 11; // ������
  } else {
    ch2 = 11; // ������
    ch1 = 11; // ������
  };
} 
