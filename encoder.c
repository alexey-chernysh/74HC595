
/**
  ******************************************************************************
  * @file encoder.c
  * @brief rotary encoder driver subroutines
  * @author Alexey Chernysh
  * @version V0.2
  * @date 09-09-2016
  ******************************************************************************
  */

#include "common.h"
#include "eeprom.h"
#include "encoder.h"
#include "serialLED.h"

static unsigned char value2display;

unsigned char GetValue2Display(){
  return value2display;
};

void SetValue2Display(unsigned char new_value){
  value2display = new_value;
};

static unsigned char ab = 0; /* 4-х битное значение датчика положения: 
                                биты 1 и 2 - текущее положение, 
                                биты 3 и 4 предыдущее положение */
                                  
unsigned char GetEncoderSensorsState();
signed char GetAngleChange(unsigned char newState);

/**
  ******************************************************************************
  * @brief SetupEncoder function.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */

void SetupEncoder(){
  value2display = 0;

  // encoder pins  
  PD_DDR_bit.DDR5 = 0;  // Ножка PD5 конфигурируется на ввод - датчик положения 1 энкодера
  PD_CR1_bit.C15 = 1;   // Выход плавающий - установливаем подтягивающий резистор
  PD_CR2_bit.C25 = 1;   // Прерывание включено
  
  PD_DDR_bit.DDR6 = 0;  // Ножка PD6 конфигурируется на ввод - датчик положения 2 энкодера
  PD_CR1_bit.C16 = 1;   // Подключено к выходу энкодера с резистором смещения - плавающий вход
  PD_CR2_bit.C26 = 1;   // Прерывание включено

  PD_DDR_bit.DDR2 = 0;  // Ножка PD2 конфигурируется на ввод - кнопка энкодера
  PD_CR1_bit.C12 = 1;   // Подключено к выходу энкодера с резистором смещения - плавающий вход
  PD_CR2_bit.C22 = 1;   // Прерывание включено
  
  EXTI_CR1_PDIS  = 3;    // Прерывания по переднему и заднему фронту для порта D

  unsigned char state = GetEncoderSensorsState();
  signed char change = GetAngleChange(state);
}

static bool delay_counter_on = false;
static unsigned int delay_counter = 0;
static unsigned int delay_counter_limit = 0xFFFF;

void StartDelayCounter(unsigned int delay_limit){
  delay_counter_limit = delay_limit;
  delay_counter_on = true;
}

void StopNResetDelayCounter(){
  delay_counter_on = false;
  delay_counter = 0;
}

void IncDelayCounter(){
  if(delay_counter_on){
    delay_counter++;
    if(delay_counter >= delay_counter_limit){
      StopNResetDelayCounter();
      ResetParamDisplayCounter();
      SetValue2Display(4);
    }
  }
}

const static signed char table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; 
                             /* шаблон значений изменений угла 
                                по текущему значению 4-х битного состаяния */

unsigned char GetEncoderSensorsState(){
  unsigned char result = PD_IDR;
  result = result >> 5;
  result = (result & 0x3);
  return result;
}

signed char GetAngleChange(unsigned char newState){
  /* increment of angle for the 16 possible bit codes */
  ab  = ab << 2; /* move the old data left two places */
  ab |= (newState & 0x3); /* OR in the two new bits */
  return table[(ab & 0xf)]; /* get the change from the 16 entry table */ 
}

static bool buttonPressed = false;

void handleButton(){
  bool buttonState = !PD_IDR_bit.IDR2;
  if(buttonState){
    if(!buttonPressed){
      StartDelayCounter(1000);
      buttonPressed = true;
      switch(value2display){
      case 0:
        value2display = 1;
        break;
      case 1:
        value2display = 2;
        break;
      case 2:
        value2display = 0;
        break;
      default:
        value2display = 0;
      }
    }
  } else {
    if(buttonPressed) buttonPressed = false;
    StopNResetDelayCounter();
  };
}

//
//  Обработка прерываний 
//

static int encoder_change_accum = 0;

#pragma vector = 8
__interrupt void EXTI_PORTD_IRQHandler(void){
  handleButton();
  unsigned char sensorsState = GetEncoderSensorsState();
  encoder_change_accum = encoder_change_accum + GetAngleChange(sensorsState);
  int change = encoder_change_accum/3;
  if(change != 0){
    encoder_change_accum = 0;
    ResetParamDisplayCounter();
    int tmp;
    switch(value2display){
    case 0:
      break;
    case 1:
      tmp = RestoreVoltageSettingFromEEPROM() + change;
      if(tmp >= 0) 
        if(tmp <= 255) StoreVoltageSettingInEEPROM((unsigned char)tmp);
      break;
    case 2:
      tmp = RestoreInitialHeightSettingFromEEPROM() + change;
      if(tmp >= 0) 
        if(tmp <= 255) StoreInitialHeightSettingInEEPROM((unsigned char)tmp);
      break;
    case 3:
      tmp = RestoreOxyfuelLiftSlowingSettingFromEEPROM() + change;
      if(tmp >= 0) 
        if(tmp <= 255) StoreOxyfuelLiftSlowingSettingInEEPROM((unsigned char)tmp);
      break;
    case 4:
      tmp = RestoreLiftVelocitySettingFromEEPROM() + change;
      if(tmp >= 0) 
        if(tmp <= 126) StoreLiftVelocitySettingInEEPROM((unsigned char)tmp);
      break;
    default:
      break;
    }
  }
}

