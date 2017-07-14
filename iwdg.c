/*
*  iwdg.c
*/

#include "common.h"

void SetupIWDG(){
  IWDG_KR = 0xcc;         //  Start the independent watchdog.
  IWDG_KR = 0x55;         //  Allow the IWDG registers to be programmed.
  IWDG_PR = 0x06;         //  Prescaler is 6 => each count is near 0.5 s
  IWDG_RLR = 0xff;        //  Reload counter.
  IWDG_KR = 0x00;         //  Write protect the IWDG registers.
  IWDG_KR = 0xaa;         //  Reset the counter.
}

void GiveBoneIWDG(){
  IWDG_KR = 0xaa;         //  Reset the counter.
}