
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

static unsigned char value2display;

unsigned char GetValue2Display(){
  return value2display;
};

void SetValue2Display(unsigned char new_value){
  value2display = new_value;
};

static unsigned char ab = 0; /* 4-� ������ �������� ������� ���������: 
                                ���� 1 � 2 - ������� ���������, 
                                ���� 3 � 4 ���������� ��������� */
                                  
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

void SetupEncoder(){
  value2display = 0;

  // encoder pins  
  PD_DDR_bit.DDR5 = 0;  // ����� PD5 ��������������� �� ���� - ������ ��������� 1 ��������
  PD_CR1_bit.C15 = 1;   // ����� ��������� - ������������� ������������� ��������
  PD_CR2_bit.C25 = 1;   // ���������� ��������
  
  PD_DDR_bit.DDR6 = 0;  // ����� PD6 ��������������� �� ���� - ������ ��������� 2 ��������
  PD_CR1_bit.C16 = 1;   // ���������� � ������ �������� � ���������� �������� - ��������� ����
  PD_CR2_bit.C26 = 1;   // ���������� ��������

  PD_DDR_bit.DDR2 = 0;  // ����� PD2 ��������������� �� ���� - ������ ��������
  PD_CR1_bit.C12 = 1;   // ���������� � ������ �������� � ���������� �������� - ��������� ����
  PD_CR2_bit.C22 = 1;   // ���������� ��������
  
  EXTI_CR1_PDIS  = 3;    // ���������� �� ��������� � ������� ������ ��� ����� D

  unsigned char state = GetEncoderSensorsState();
  signed char change = GetAngleChange(state);
}

const static signed char table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; 
                             /* ������ �������� ��������� ���� 
                                �� �������� �������� 4-� ������� ��������� */

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
  bool buttonState = PD_IDR_bit.IDR2;
  if(buttonState){
    if(!buttonPressed){
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
  };
}

//
//  ��������� ���������� 
//

static int encoder_change_accum = 0;

#pragma vector = 8
__interrupt void EXTI_PORTD_IRQHandler(void)
{
  handleButton();
  unsigned char sensorsState = GetEncoderSensorsState();
  encoder_change_accum = encoder_change_accum + GetAngleChange(sensorsState);
  int change = encoder_change_accum/3;
  if(change != 0){
    encoder_change_accum = 0;
    int tmp;
    switch(value2display){
    case 0:
      value2display = 1;
      break;
    case 1:
      tmp = GetAVS() + change;
      if(tmp >= 0) 
        if(tmp <= 255) SetAVS((unsigned char)tmp);
      break;
    case 2:
      tmp = GetIHS() + change;
      if(tmp >= 0) 
        if(tmp <= 255) SetIHS((unsigned char)tmp);
      break;
    case 3:
      tmp = GetTLS() + change;
      if(tmp >= 0) 
        if(tmp <= 127) SetTLS((unsigned char)tmp);
      break;
    default:
      break;
    }
  }
}

