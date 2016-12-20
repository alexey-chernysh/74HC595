/**
  ******************************************************************************
  * @file clock.c
  * @brief STM8S clock initialization 
  * @author Alexey Chernysh
  * @version V0.2
  * @date 09-09-2016
  ******************************************************************************
  */

#include "common.h"

/**
  ******************************************************************************
  * @brief SetupClock function.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */

void SetupClock(){
  CLK_ICKR = 0;       // СБрос внутреннего источника тактирования
  CLK_ICKR_HSIEN = 1; // Включаем тактирование от внутреннего генратора HSI
  CLK_ECKR = 0;       // Отключаем внешний генератор
  while (CLK_ICKR_HSIRDY == 0); // Ждём стабилизации внутреннего источника тактирования
  CLK_CKDIVR = 0;     // Делитель частоты не используем
  CLK_PCKENR1 = 0xff; // Включаем тактирование всей переферии
  CLK_PCKENR2 = 0xff; // Включаем тактирование всей переферии
  CLK_CCOR = 0;
  CLK_HSITRIMR = 0;
  CLK_SWIMCCR = 0;
  CLK_SWR = 0xe1;
  CLK_SWCR = 0;
  CLK_SWCR_SWEN = 1;  // Включаем тактирование
  while (CLK_SWCR_SWBSY != 0);
}
