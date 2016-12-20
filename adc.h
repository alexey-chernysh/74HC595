#ifndef __ADC_H__
#define __ADC_H__

void SetupADC();
void RestartADC(void);
unsigned char GetCurrentVoltage();
void setInitialHeight(char newValue);

#endif /*__ADC_H_*/
