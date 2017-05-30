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

  // Настройка АЦП на канале АIN4/PB4
  ADC_CR1_ADON = 1;       //  Первое включее АЦП
  ADC_CR1_CONT = 0;
  ADC_CR2_ALIGN = 0;      //  Выравнивание по левому краю
  ADC_CR3_DBUF = 0;       //  без буферизации
  ADC_CSR_CH = 0x04;      //  АЦП на слаботочном канале AIN4 
  ADC_CSR_EOCIE = 1;      //  Включаем прерывание по завершению пребразования АЦП.
  ADC_CR1_ADON = 1;       //  перезапуск АЦП
  
  GetIHS();
}

void RestartADC(void){
  ADC_CR1_ADON = 1;       //  перезапуск АЦП
}

static const unsigned char VOLTAGE_LOW_LIMIT = 40;
static const unsigned char VOLTAGE_HIGH_LIMIT = 210;
static unsigned int current_voltage = 0;

unsigned char GetCurrentVoltage(){ return current_voltage;}

#define UP_SIGNAL                   PA_IDR_bit.IDR1
#define DOWN_SIGNAL                 PA_IDR_bit.IDR2
#define INITIAL_POSITIONING_BUTTON  PB_IDR_bit.IDR3
#define PREHEAT_SIGNAL              PC_IDR_bit.IDR1
#define INITIAL_POSITIONING_SIGNAL  PD_IDR_bit.IDR0
#define COLLISION_SIGNAL            PD_IDR_bit.IDR7
#define POSITIONING_COMPLETE_SIGNAL PE_ODR_bit.ODR5
#define AUTO_SIGNAL                 PF_IDR_bit.IDR4

static signed long up_after_collision_counter = 0;
static signed long up_after_collision_counter_limit = 50000L;
static signed long down_for_plate_collision_counter = 0;
static signed long down_for_plate_collision_counter_limit = 500000L;
static bool preheat = false;

bool preheatOn(){
  return preheat;
}

unsigned char sensetivity;
void setSensetivity(unsigned char s){
  sensetivity = s;
}

void setInitialHeight(char newValue){
  up_after_collision_counter_limit = 200L * newValue;
}

void delay(){
  int j=0;
  for(int i=0; i<300; i++){
    j++;
  }
}

static signed int upVelocity = 126;
static signed int downVelocity = -126;

signed int GetLiftMotionVelocity(signed int current_delta){
  signed int current_lift_motion_velocity = 0;
  
  if(PREHEAT_SIGNAL == 0) preheat = true;
  else preheat = false;
  
  if(AUTO_SIGNAL == 0) 
    current_lift_motion_velocity = current_delta;
  
  POSITIONING_COMPLETE_SIGNAL = 1; // сбросить сигнал завершения "теста на касание"

  // обработка команды на начальное позиционирование
  bool isInitialPositioning = ((INITIAL_POSITIONING_SIGNAL == 0)||(INITIAL_POSITIONING_BUTTON == 0));
  if(isInitialPositioning){
    // если есть сигнал "касание" от контроллера или кнопки
    down_for_plate_collision_counter = down_for_plate_collision_counter_limit; // устанавливаем счетчик на величину, пропорционяльную таймауту
  }

  if(down_for_plate_collision_counter > 0) { // вниз, пока не случится коллизия или не обнулится счетяик
    current_lift_motion_velocity = downVelocity; // вниз на максимальной скорости
    down_for_plate_collision_counter--;  // уменшаем счетчик ожидания таймаута
  }
  
  // обрабатываем значание датчика прикосновения
  if(COLLISION_SIGNAL > 0){ // сработка датчика прикосновения с листом
    down_for_plate_collision_counter = 0;  // завершаем движение вниз
    if(up_after_collision_counter <= 0) 
      up_after_collision_counter = up_after_collision_counter_limit; // начинаем движение вверх после касания/коллизии
  }; 

  if(up_after_collision_counter > 0) { // вверх, в течении up_after_collision_counter_limit циклов
    current_lift_motion_velocity = upVelocity;  // вверх, с максимальной скоростью
    up_after_collision_counter--;
    if(up_after_collision_counter == 0){
      POSITIONING_COMPLETE_SIGNAL = 0; // выдать сигнал завершения "теста на касание"
      delay();
    }
  }
  
  // обработка сигналов контроллера "резак вверх" и "резак вниз"
  if(DOWN_SIGNAL == 0)
    if(preheatOn()) current_lift_motion_velocity = -sensetivity; // вниз, с уставкой
    else current_lift_motion_velocity = downVelocity; // вниз, с максимальной скоростью
  if(UP_SIGNAL == 0)   
    if(preheatOn()) current_lift_motion_velocity = sensetivity; // вверх, с уставкой
    else current_lift_motion_velocity = upVelocity; // вверх, с максимальной скоростью 

  return current_lift_motion_velocity;
}

#pragma vector = ADC1_EOC_vector
__interrupt void ADC1_EOC_IRQHandler(){
  if(ADC_CSR_EOC == 1){
    unsigned int low, high;
    low  = ADC_DRL;  //    Extract the ADC low byte
    high = ADC_DRH;  //    Extract the ADC hi byte
    current_voltage = (high<<8) + low;
    current_voltage = current_voltage>>7;
    signed int delta = 0;
    if(current_voltage > VOLTAGE_LOW_LIMIT)
      if(current_voltage < VOLTAGE_HIGH_LIMIT)
        delta = GetAVS() - current_voltage;
    delta = GetLiftMotionVelocity(delta<<3);
    SetMotorVelocity(delta); // вверх или вниз, со скоростью delta
  }
}
