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
  PD_DDR_bit.DDR0 = 0;  // ����� PD0 ��������������� �� ���� - ������ "���� �� �������" �� �����������
                        // �� �� ����� - ���/���������� - ����
  PD_CR1_bit.C10 = 1;   // ����� ��������� - ������������� ������������� ��������
  PD_CR2_bit.C20 = 0;   // ���������� ���������

  PD_DDR_bit.DDR7 = 0;  // ����� PD7 ��������������� �� ���� - ������ "�������" �� ���������  / top level interrupt
                        // �� �� ����� - ���/���������� - ����
  PD_CR1_bit.C17 = 1;   // ����� ��������� - ������������� ������������� ��������
  PD_CR2_bit.C27 = 0;   // ���������� ���������

  PA_DDR_bit.DDR1 = 0;  // ����� PA1 ��������������� �� ���� - ������ "����� �����" �� �����������
                        // �� �� ����� - ���/���������� - ����
  PA_CR1_bit.C11 = 1;   // ����� ��������� - ������������� ������������� ��������  EXTI_CR1_PAIS  = 3;    // ���������� �� ��������� � ������� ������ ��� ����� A
  PA_CR2_bit.C21 = 0;   // ���������� ���������

  PA_DDR_bit.DDR2 = 0;  // ����� PA2 ��������������� �� ���� - ������ "����� ����" �� �����������
                        // �� �� ����� - ���/���������� - ����
  PA_CR1_bit.C12 = 1;   // ����� ��������� - ������������� ������������� ��������
  PA_CR2_bit.C22 = 0;   // ���������� ���������

  PF_DDR_bit.DDR4 = 0;  // ����� PF4 ��������������� �� ���� - ���� ���������� �������� ������ 
                        // �� �� ����� - ���/���������� - ����
  PF_CR1_bit.C14 = 1;   // ����� ��������� - ������������� ������������� ��������
  PF_CR2_bit.C24 = 0;   // ���������� ���������

  PE_DDR_bit.DDR5 = 1;  // ����� PE5 ��������������� �� ����� - ����� ���������� ���������� ���������������� 
                        // �� �� ����� - ���/���������� - ����
  PE_CR1_bit.C15 = 1;   // ����� ��������� - ������������� ������������� ��������
  PE_CR2_bit.C25 = 0;   // ���������� ���������

  // ��������� ��� �� ������ �IN4/PB4
  ADC_CR1_ADON = 1;       //  ������ ������� ���
  ADC_CR1_CONT = 0;
  ADC_CR2_ALIGN = 0;      //  ������������ �� ������ ����
  ADC_CR3_DBUF = 0;       //  ��� �����������
  ADC_CSR_CH = 0x04;      //  ��� �� ����������� ������ AIN4 
  ADC_CSR_EOCIE = 1;      //  �������� ���������� �� ���������� ������������� ���.
  ADC_CR1_ADON = 1;       //  ���������� ���
  
  GetIHS();
}

void RestartADC(void){
  ADC_CR1_ADON = 1;       //  ���������� ���
}

static const unsigned char VOLTAGE_LOW_LIMIT = 50;
static const unsigned char VOLTAGE_HIGH_LIMIT = 210;
static unsigned char current_voltage = 0;

unsigned char GetCurrentVoltage(){ return current_voltage;}

#define INITIAL_POSITIONING_SIGNAL  PD_IDR_bit.IDR0
#define COLLISION_SIGNAL            PD_IDR_bit.IDR7
#define UP_SIGNAL                   PA_IDR_bit.IDR1
#define DOWN_SIGNAL                 PA_IDR_bit.IDR2
#define AUTO_SIGNAL                 PF_IDR_bit.IDR4
#define POSITIONING_COMPLETE_SIGNAL PE_ODR_bit.ODR5

static signed long up_after_collision_counter = 0;
static unsigned long up_after_collision_counter_limit = 50000;

void setInitialHeight(char newValue){
  up_after_collision_counter_limit = 200L * newValue;
}


signed char GetLiftMotionVelocity(signed char current_delta){
  signed char current_lift_motion_velocity = 0;
  
  if(AUTO_SIGNAL == 0) 
    current_lift_motion_velocity = current_delta;
  
  POSITIONING_COMPLETE_SIGNAL = 1; // �������� ������ ���������� "����� �� �������"

  // ��������� ������� �� ��������� ����������������
  if(INITIAL_POSITIONING_SIGNAL == 0) // ���� ���� ������ "�������" �� �����������
    current_lift_motion_velocity = -127; // ���� �� ������������ ��������
  
  // ������������ �������� ������� �������������
  if(COLLISION_SIGNAL > 0){ // �������� ������� ������������� � ������
    if(up_after_collision_counter <= 0) 
      up_after_collision_counter = up_after_collision_counter_limit; // �������� �������� ����� ����� �������/��������
    if(INITIAL_POSITIONING_SIGNAL == 0) // ���� ������� ��������� "���� �� �������"
      POSITIONING_COMPLETE_SIGNAL = 0; // ������ ������ ���������� "����� �� �������"
  }; 

  if(up_after_collision_counter > 0) { // �����, � ������� up_after_collision_counter_limit ������
    current_lift_motion_velocity = 127;  // �����, � ������������ ���������
    up_after_collision_counter--;
  }
  
  // ��������� �������� ����������� "����� �����" � "����� ����"
  if(DOWN_SIGNAL == 0) 
    current_lift_motion_velocity = -127; // ����, � ������������ ���������
  if(UP_SIGNAL == 0)   
    current_lift_motion_velocity =  127; // �����, � ������������ ��������� 

  return current_lift_motion_velocity;
}

#pragma vector = ADC1_EOC_vector
__interrupt void ADC1_EOC_IRQHandler(){
  if(ADC_CSR_EOC == 1){
    current_voltage = ADC_DRH; //    Extract the ADC reading.
    int tmp_delta = 0;
    if(current_voltage > VOLTAGE_LOW_LIMIT)
      if(current_voltage < VOLTAGE_HIGH_LIMIT)
        tmp_delta = GetAVS() - current_voltage;
    tmp_delta = tmp_delta<<3;
    signed char voltage_delta = (char)tmp_delta;
    if(tmp_delta >  127) voltage_delta =  127;
    if(tmp_delta < -127) voltage_delta = -127;
    voltage_delta = GetLiftMotionVelocity(voltage_delta);
    SetMotorVelocity(voltage_delta); // ����� ��� ����, �� ��������� voltage_delta
  }
}
