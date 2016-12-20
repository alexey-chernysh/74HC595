
#include "common.h"
#include "clock.h"
#include "motor_control.h"
#include "encoder.h"
#include "serialLED.h"
#include "adc.h"
#include "eeprom.h"

void Setup(){
  SetupClock();
  SetupADC();
  SetupMotorControl();
  SetupEncoder();
  SetupSerialLEDs();
}

void main( void )
{
  __disable_interrupt();
  Setup();
  __enable_interrupt();
    
  do{ // Бесконечный цикл
    __wait_for_interrupt();
  } while(true); 

}
