/**
  ******************************************************************************
  * @file adc.c
  * @brief STM8S analog to digital converter and digital I/O subroutines 
  * @author Alexey Chernysh
  * @version V0.2
  * @date 09-09-2016
  ******************************************************************************
  */

#include "common.h"
#include "iwdg.h"
#include "eeprom.h"
#include "motor_control.h"

/**
  ******************************************************************************
  * @brief SetupADC function.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */

void SetupADC(){
  
  // controller signal pins
  PA_DDR_bit.DDR1 = 0;  // Ножка PA1 конфигурируется на ввод - сигнал "резак вверх" от контроллера
                        // КЗ на землю - вкл/разомкнуто - выкл
  PA_CR1_bit.C11 = 1;   // Выход плавающий - установливаем подтягивающий резисто
  PA_CR2_bit.C21 = 0;   // Прерывание выключено

  PA_DDR_bit.DDR2 = 0;  // Ножка PA2 конфигурируется на ввод - сигнал "резак вниз" от контроллера
                        // КЗ на землю - вкл/разомкнуто - выкл
  PA_CR1_bit.C12 = 1;   // Выход плавающий - установливаем подтягивающий резистор
  PA_CR2_bit.C22 = 0;   // Прерывание выключено

  PB_DDR_bit.DDR3 = 0;  // Ножка PB3 конфигурируется на ввод - сигнал кнопки "касание" 
                        // КЗ на землю - вкл/разомкнуто - выкл
  PB_CR1_bit.C13 = 1;   // Выход плавающий - установливаем подтягивающий резистор
  PB_CR2_bit.C23 = 0;   // Прерывание выключено

  PC_DDR_bit.DDR1 = 0;  // Ножка PC1 конфигурируется на ввод - сигнал "прогрев" 
                        // КЗ на землю - вкл/разомкнуто - выкл
  PC_CR1_bit.C11 = 1;   // Выход плавающий - установливаем подтягивающий резистор
  PC_CR2_bit.C21 = 0;   // Прерывание выключено

  PD_DDR_bit.DDR0 = 0;  // Ножка PD0 конфигурируется на ввод - сигнал "вниз до касания" от контроллера
                        // КЗ на землю - вкл/разомкнуто - выкл
  PD_CR1_bit.C10 = 1;   // Выход плавающий - установливаем подтягивающий резистор
  PD_CR2_bit.C20 = 0;   // Прерывание выключено

  PD_DDR_bit.DDR7 = 0;  // Ножка PD7 конфигурируется на ввод - сигнал "касание" от концевика  / top level interrupt
                        // КЗ на землю - вкл/разомкнуто - выкл
  PD_CR1_bit.C17 = 1;   // Выход плавающий - установливаем подтягивающий резистор
  PD_CR2_bit.C27 = 0;   // Прерывание выключено

  PF_DDR_bit.DDR4 = 0;  // Ножка PF4 конфигурируется на ввод - взод разрешения контроля высоты 
                        // КЗ на землю - вкл/разомкнуто - выкл
  PF_CR1_bit.C14 = 1;   // Выход плавающий - установливаем подтягивающий резистор
  PF_CR2_bit.C24 = 0;   // Прерывание выключено

  PE_DDR_bit.DDR5 = 1;  // Ножка PE5 конфигурируется на вывод - выход завершения начального позиционирования 
                        // КЗ на землю - вкл/разомкнуто - выкл
  PE_CR1_bit.C15 = 1;   // Выход плавающий - установливаем подтягивающий резистор
  PE_CR2_bit.C25 = 0;   // Прерывание выключено

  PC_DDR_bit.DDR3 = 1;  // Ножка PC3 конфигурируется на вывод - реле 1 защиты АЦП  от заноса высокого 
                        // КЗ на землю - вкл/разомкнуто - выкл
  PC_CR1_bit.C13 = 1;   // Выход плавающий - установливаем подтягивающий резистор
  PC_CR2_bit.C23 = 0;   // Прерывание выключено

  PC_DDR_bit.DDR4 = 1;  // Ножка PC4 конфигурируется на вывод - реле 2 защиты АЦП  от заноса высокого 
                        // КЗ на землю - вкл/разомкнуто - выкл
  PC_CR1_bit.C14 = 1;   // Выход плавающий - установливаем подтягивающий резистор
  PC_CR2_bit.C24 = 0;   // Прерывание выключено

  // Настройка АЦП на канале АIN4/PB4
  ADC_CR1_ADON = 1;       //  Первое включее АЦП
  ADC_CR1_CONT = 0;
  ADC_CR2_ALIGN = 0;      //  Выравнивание по левому краю
  ADC_CR3_DBUF = 0;       //  без буферизации
  ADC_CSR_CH = 0x04;      //  АЦП на слаботочном канале AIN4 
  ADC_CSR_EOCIE = 1;      //  Включаем прерывание по завершению пребразования АЦП.
  ADC_CR1_ADON = 1;       //  перезапуск АЦП
  
}

void RestartADC(void){
  ADC_CR1_ADON = 1;       //  перезапуск АЦП
}

static const unsigned char VOLTAGE_LOW_LIMIT = 25;
static const unsigned char VOLTAGE_HIGH_LIMIT = 185;
static unsigned int current_voltage = 0;

unsigned char GetCurrentVoltage(){ return current_voltage;}

#define UP_SIGNAL                   PA_IDR_bit.IDR1
#define DOWN_SIGNAL                 PA_IDR_bit.IDR2
#define INITIAL_POSITIONING_BUTTON  PB_IDR_bit.IDR3
#define PREHEAT_SIGNAL              PC_IDR_bit.IDR1
#define INITIAL_POSITIONING_SIGNAL  PD_IDR_bit.IDR0
#define COLLISION_SIGNAL            PD_IDR_bit.IDR7
#define AUTO_SIGNAL                 PF_IDR_bit.IDR4
#define POSITIONING_COMPLETE_SIGNAL PE_ODR_bit.ODR5
#define SURGE_PROTECTION_1          PC_ODR_bit.ODR3
#define SURGE_PROTECTION_2          PC_ODR_bit.ODR4

//#define IP_TRIGGER 1

static signed long up_after_collision_counter = 0;
#ifdef IP_TRIGGER
static signed long down_for_plate_collision_counter = 0;
static signed long down_for_plate_collision_counter_limit = 500000L;
#endif
static bool waiting_for_IP_signal_released = false;
static const unsigned int IP_complete_signal_hold_delay = 1000;
static unsigned int IP_complete_signal_hold_counter = 0;

static bool preheat = false;

bool preheatOn(){
  return preheat;
}

signed int GetLiftMotionVelocity(signed int current_delta){
  signed int current_lift_motion_velocity = 0;
  
  if(PREHEAT_SIGNAL == 0) preheat = true;
  else preheat = false;
  
  bool initial_positioning_signal;
  if(INITIAL_POSITIONING_SIGNAL == 0) initial_positioning_signal = true;
  else initial_positioning_signal = false;
  
  bool initial_positioning_button;
  if(INITIAL_POSITIONING_BUTTON == 0) initial_positioning_button = true;
  else initial_positioning_button = false;
  
  bool collision_signal;
  if(COLLISION_SIGNAL > 0) collision_signal = true;
  else collision_signal = false;
  
  bool torch_down;
  if(DOWN_SIGNAL == 0) torch_down = true;
  else torch_down = false;
    
  bool torch_up;
  if(UP_SIGNAL == 0) torch_up = true;
  else torch_up = false;
    
  if(AUTO_SIGNAL == 0){ 
    current_lift_motion_velocity = current_delta;
    SURGE_PROTECTION_1 = 0;
    SURGE_PROTECTION_2 = 0;
  } else {
    SURGE_PROTECTION_1 = 1;
    SURGE_PROTECTION_2 = 1;
  }
  
  signed int upVelocity = RestoreLiftVelocitySettingFromEEPROM();
  signed int downVelocity = -upVelocity;
  
  // обработка ожидания завершения начального позиционирования
  bool isInitialPositioning = false;
  if(IP_complete_signal_hold_counter > 0) IP_complete_signal_hold_counter--;
  
  if((waiting_for_IP_signal_released)||(IP_complete_signal_hold_counter > 0)){
    // удерживаем сигнал завершения "теста на касание"
    POSITIONING_COMPLETE_SIGNAL = 0;
#ifdef IP_TRIGGER
    down_for_plate_collision_counter = 0;
#endif
    if(!initial_positioning_signal) {
      waiting_for_IP_signal_released = false;
    };
  } else {
    POSITIONING_COMPLETE_SIGNAL = 1;
    if(initial_positioning_signal) 
      isInitialPositioning = true;
    if(initial_positioning_button) 
      isInitialPositioning = true;
  };

#ifdef IP_TRIGGER
  if(isInitialPositioning){
    // если есть сигнал "касание" от контроллера или кнопки
    // устанавливаем счетчик на величину, пропорционяльную таймауту
    down_for_plate_collision_counter = down_for_plate_collision_counter_limit; 
  };

  if(down_for_plate_collision_counter > 0) { // вниз, пока не случится коллизия или не обнулится счетчик
    current_lift_motion_velocity = downVelocity; // вниз на максимальной скорости
    down_for_plate_collision_counter--;  // уменшаем счетчик ожидания таймаута
  }
  
#else

  if(initial_positioning_signal) 
    isInitialPositioning = true;
  if(initial_positioning_button) 
    isInitialPositioning = true;
  if(isInitialPositioning) 
    current_lift_motion_velocity = downVelocity; // вниз на максимальной скорости

#endif

  // обработка сигналов контроллера "резак вниз"
  if(torch_down)
    if(preheat) current_lift_motion_velocity = - RestoreOxyfuelLiftSlowingSettingFromEEPROM(); // вниз, с уставкой
    else current_lift_motion_velocity = downVelocity; // вниз, с максимальной скоростью

    // обрабатываем значание датчика прикосновения
  if(collision_signal){ // сработка датчика прикосновения с листом
#ifdef IP_TRIGGER
    // завершаем движение вниз
    down_for_plate_collision_counter = 0; 
#endif
    // начинаем движение вверх после касания/коллизии
    up_after_collision_counter = 100L*RestoreInitialHeightSettingFromEEPROM(); 
  }; 

  if(up_after_collision_counter > 0) { // вверх, в течении up_after_collision_counter_limit циклов
    current_lift_motion_velocity = upVelocity;  // вверх, с максимальной скоростью
    up_after_collision_counter--;
    if(up_after_collision_counter <= 0){
      current_lift_motion_velocity = 0; // остановить подъем после коллизии
      waiting_for_IP_signal_released = true; // переходим в ожидание снятия сигнала начального позиционирования
      // удерживаем сигнал "IP_COMPLETE"
      IP_complete_signal_hold_counter = IP_complete_signal_hold_delay;
    } 
  }
  
  // обработка сигналов контроллера "резак вверх"
  if(torch_up)   
    if(preheat) current_lift_motion_velocity = RestoreOxyfuelLiftSlowingSettingFromEEPROM(); // вверх, с уставкой
    else current_lift_motion_velocity = upVelocity; // вверх, с максимальной скоростью 

  return current_lift_motion_velocity;
}

#define GATE 2

#pragma vector = ADC1_EOC_vector
__interrupt void ADC1_EOC_IRQHandler(){
  if(ADC_CSR_EOC == 1){
    GiveBoneIWDG();
    unsigned int low, high;
    low  = ADC_DRL;  //    Extract the ADC low byte
    high = ADC_DRH;  //    Extract the ADC hi byte
    current_voltage = (high<<8) + low;
//    current_voltage = current_voltage>>7;  // 1/100 voltage divider
    current_voltage = current_voltage>>8;  // 1/50 voltage divider
    signed int delta = 0;
    
    if(current_voltage > VOLTAGE_LOW_LIMIT)
      if(current_voltage < VOLTAGE_HIGH_LIMIT)
        delta = RestoreVoltageSettingFromEEPROM() - current_voltage;

    if( delta < GATE ){
      if( delta > (-GATE) ) delta = 0;
      else delta = delta + GATE;
    } else delta = delta - GATE;

    delta = GetLiftMotionVelocity(3*delta);  // для консольки надо <<1
    SetMotorVelocity(delta); // вверх или вниз, со скоростью delta
    RestartADC();
  }
}
