
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

// PD3/TIM2_CH2 �� IN2 TLE 
// PD4/TIM2_CH1 �� IN1 TLE 

// ��� ����������� TLE5206
// PD3/TIM2_CH2  - ��� ���2 
// PD4/TIM2_CH1  - ��� ���1 

#define TLE5205 1
// ��� ����������������� TLE5206 (TLE5205)
// PD3/TIM2_CH2  - ��� DIR 
// PD4/TIM2_CH1  - ��� ��� 

#define PWM_LIMIT 90    
    
void SetupMotorControl()
{
  PD_DDR_bit.DDR3 = 1;   // ����� PD3 ��������������� �� �����
  PD_CR1_bit.C13 = 1;    // ����� ���� Push-pull
  PD_CR2_bit.C23 = 1;    // �������� ������������ - �� 10 ���.
  PD_ODR_bit.ODR3 = 0;
  
  PD_DDR_bit.DDR4 = 1;   // ����� PD4 ��������������� �� �����
  PD_CR1_bit.C14 = 1;    // ����� ���� Push-pull
  PD_CR2_bit.C24 = 1;    // �������� ������������ - �� 10 ���.
  PD_ODR_bit.ODR4 = 0;

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

#ifdef TLE5205
  // ��� TLE5205 - ������ ��� ��� ����������� � �� �� ������������
  TIM2_CCR1H = 0;
  TIM2_CCR1L = 0;
  TIM2_CCER1_CC1P = 1;    
  TIM2_CCER1_CC1E = 1;    
  TIM2_CCMR1_OC1M = 6;    
#else
  // ��� TLE5206 - ������ ��� ��� ����� ��� ��� �������� �����
  TIM2_CCR1H = 0;
  TIM2_CCR1L = 0;
  TIM2_CCER1_CC1P = 1;    //  Active high.
  TIM2_CCER1_CC1E = 1;    //  Enable compare mode for channel 1
  TIM2_CCMR1_OC1M = 6;    //  PWM Mode 1 - active if counter < CCR1, inactive otherwise.
#endif

#ifdef TLE5205
  // ��� TLE5205 - ������ ��� ��� ��� � ������� ������ �������
  TIM2_CCR2H = 0;
  TIM2_CCR2L = 0;
  TIM2_CCER1_CC2P = 0;    //  Active high.
  TIM2_CCER1_CC2E = 0;    //  Enable compare mode for channel 2
  TIM2_CCMR2_OC2M = 0;    //  PWM Mode 1 - active if counter < CCR2, inactive otherwise.
#else
  // ��� TLE5206 - ������ ��� ��� ����� ��� ��� �������� ����
  TIM2_CCR2H = 0;
  TIM2_CCR2L = 0;
  TIM2_CCER1_CC2P = 1;    //  Active high.
  TIM2_CCER1_CC2E = 1;    //  Enable compare mode for channel 2
  TIM2_CCMR2_OC2M = 6;    //  PWM Mode 1 - active if counter < CCR2, inactive otherwise.
#endif

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
#define GATE 5

void SetMotorVelocity(signed int pwm){

  unsigned int tmp;

  if(pwm >= 0) tmp = pwm;
  else tmp = -pwm;

  unsigned char high = (unsigned char)(tmp>>8);
  unsigned char low = (unsigned char)(tmp);

#ifdef TLE5205
  TIM2_CCR1H = high;
  TIM2_CCR1L = low;
  if(pwm >= 0){
    PD_ODR_bit.ODR3 = 0;
  } else {
    PD_ODR_bit.ODR3 = 1;
  };
#else
  if(pwm >= 0){
    TIM2_CCR1H = high;
    TIM2_CCR1L = low;
    TIM2_CCR2H = 0;
    TIM2_CCR2L = 0;
  } else {
    TIM2_CCR1H = 0;
    TIM2_CCR1L = 0;
    TIM2_CCR2H = high;
    TIM2_CCR2L = low;
  };
#endif

}
