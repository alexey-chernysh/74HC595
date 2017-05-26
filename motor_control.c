
/**
  ******************************************************************************
  * @file motror_control.c
  * @brief DC motor PWM driver subroutines
  * @author Alexey Chernysh
  * @version V0.2
  * @date 09-09-2016
  ******************************************************************************
  */

#include "common.h"

/**
  ******************************************************************************
  * @brief SetupMotorControl function.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */

#define PWM_LIMIT 70    
    
void SetupMotorControl()
{
  PD_DDR_bit.DDR3 = 1;   // ����� PD3 ��������������� �� �����
  PD_CR1_bit.C13 = 1;    // ����� ���� Push-pull
  PD_CR2_bit.C23 = 1;    // �������� ������������ - �� 10 ���.
  PD_ODR_bit.ODR3 = 1;
  
  PD_DDR_bit.DDR4 = 1;   // ����� PD4 ��������������� �� �����
  PD_CR1_bit.C14 = 1;    // ����� ���� Push-pull
  PD_CR2_bit.C24 = 1;    // �������� ������������ - �� 10 ���.
  PD_ODR_bit.ODR4 = 1;

  //�������� ������� 2

  TIM2_CR1 = 0; // Turn off timer
  TIM2_IER = 0; // disable interrupts
  TIM2_SR1 = 0; // reset status
  TIM2_CCMR1 = 0;
  TIM2_CCMR2 = 0;
  TIM2_CCMR3 = 0;
  TIM2_CCER1 = 0;
  TIM2_CCER2 = 0;

  TIM2_PSCR = 6;       // �������� 1
  TIM2_ARRH = (unsigned char)((PWM_LIMIT)>>8); // ������� ���� ������� �������� ����� ���
  TIM2_ARRL = (unsigned char)(PWM_LIMIT);      // ������� ���� ������� �������� ����� ���

  TIM2_CCR1H = 0;
  TIM2_CCR1L = 0;
  TIM2_CCER1_CC1P = 1;    //  Active high.
  TIM2_CCER1_CC1E = 1;    //  Enable compare mode for channel 1
  TIM2_CCMR1_OC1M = 6;    //  PWM Mode 1 - active if counter < CCR1, inactive otherwise.

  TIM2_CCR2H = 0;
  TIM2_CCR2L = 0;
  TIM2_CCER1_CC2P = 1;    //  Active high.
  TIM2_CCER1_CC2E = 1;    //  Enable compare mode for channel 2
  TIM2_CCMR2_OC2M = 6;    //  PWM Mode 1 - active if counter < CCR2, inactive otherwise.
  
  TIM2_CR1_CEN = 1;       //  Finally enable the timer.

}

/**
  ******************************************************************************
  * @brief SetMotorRotation function.
  * @par Parameters:
  * val - �������� �����, ����������, ��� ��������� ����� �������� ����� ���
  * @retval void None
  * @par Required preconditions:
  * ������ val <= 127 
  ******************************************************************************
  */
void SetMotorVelocity(signed int pwm){

  unsigned char high;
  unsigned char low;

  if(pwm >= 0){
    high = (unsigned char)(pwm>>8);
    low = (unsigned char)(pwm);
    TIM2_CCR1H = high;
    TIM2_CCR1L = low;
    TIM2_CCR2H = 0;
    TIM2_CCR2L = 0;
  } else {
    high = (unsigned char)((-pwm)>>8);
    low = (unsigned char)(-pwm);
    TIM2_CCR1H = 0;
    TIM2_CCR1L = 0;
    TIM2_CCR2H = high;
    TIM2_CCR2L = low;
  };

}
