
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdbool.h> // ����������� ������������� ����� ��� ������������ ���� bool

void SetupEncoder();
unsigned char GetValue2Display();
void SetValue2Display(unsigned char new_value);
void IncDelayCounter();

#endif /*__ENCODER_H_*/
