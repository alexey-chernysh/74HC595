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
  CLK_ICKR = 0;       // ����� ����������� ��������� ������������
  CLK_ICKR_HSIEN = 1; // �������� ������������ �� ����������� ��������� HSI
  CLK_ECKR = 0;       // ��������� ������� ���������
  while (CLK_ICKR_HSIRDY == 0); // ��� ������������ ����������� ��������� ������������
  CLK_CKDIVR = 0;     // �������� ������� �� ����������
  CLK_PCKENR1 = 0xff; // �������� ������������ ���� ���������
  CLK_PCKENR2 = 0xff; // �������� ������������ ���� ���������
  CLK_CCOR = 0;
  CLK_HSITRIMR = 0;
  CLK_SWIMCCR = 0;
  CLK_SWR = 0xe1;
  CLK_SWCR = 0;
  CLK_SWCR_SWEN = 1;  // �������� ������������
  while (CLK_SWCR_SWBSY != 0);
}
