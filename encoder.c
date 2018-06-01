
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

const unsigned int TIM1_PRESCALER = 15999;
const unsigned int TIM1_LIMIT = 9999;

void SetupTimer1(){
  //Насройка таймера 1
  TIM1_CR1 = 0; // ВЫключаем таймкр для программирования
  TIM1_CR2 = 0; // Синхронизация как ведущий с периферией отключена
  TIM1_IER = 0; // Блокируем прерывание 
  TIM1_SR1 = 0; // Сбрасываем состояние
  TIM1_SMCR = 0;// Синхронизация как ведомый с периферией отключена
  TIM1_ETR = 0; // Внешнее тактирование отключено
    
  TIM1_PSCRH = (unsigned char)(TIM1_PRESCALER>>8);
  TIM1_PSCRL = (unsigned char)(TIM1_PRESCALER);
  TIM1_ARRH = (unsigned char)(TIM1_LIMIT>>8); // Старший байт предела счетчика 
  TIM1_ARRL = (unsigned char)(TIM1_LIMIT);    // Младший байт предела счетчика
  TIM1_CCR1H = 0xFF;
  TIM1_CCR1L = 0xFF;
  TIM1_CCR2H = 0xFF;
  TIM1_CCR2L = 0xFF;
  TIM1_CCR3H = 0xFF;
  TIM1_CCR3L = 0xFF;
  TIM1_CCR4H = 0xFF;
  TIM1_CCR4L = 0xFF;
}

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
 
  SetupTimer1();
}

//const unsigned short repetition = 2;

void StartTimer1(){
  TIM1_CR1 = 0; 
//  TIM1_CR1_OPM = 1; // One Pulse Mode enabled
  TIM1_CR1_URS = 1;
  TIM1_CR1_DIR = 1;
//  TIM1_RCR = repetition;
  TIM1_IER_UIE = 1; // Разрешаем прерывание по переполнению
  TIM1_CR1_CEN = 1; // Выключаем таймер
}

void StopNResetTimer1(){
  TIM1_SR1_UIF = 0;
  TIM1_CR1 = 0; // Выключаем таймер
  TIM1_SR1 = 0; // Сбрасываем состояние
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
      StartTimer1();
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
    StopNResetTimer1();
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
        if(tmp <= 127) StoreOxyfuelLiftSlowingSettingInEEPROM((unsigned char)tmp);
      break;
    case 4:
      tmp = RestoreLiftVelocitySettingFromEEPROM() + change;
      if(tmp >= 0) 
        if(tmp <= 127) StoreLiftVelocitySettingInEEPROM((unsigned char)tmp);
      break;
    default:
      break;
    }
  }
}

// Вектор прерывания по обновлению или переполнению Таймера1
#pragma vector = TIM1_OVR_UIF_vector
__interrupt void TIM1_OVR_UIF_handler(void){
  // Проверка, что же вызвало прерывание
  if (TIM1_SR1_UIF == 1){
    StopNResetTimer1();
    SetValue2Display(4);
  }  
}
