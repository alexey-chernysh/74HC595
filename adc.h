#ifndef __ADC_H__
#define __ADC_H__

bool preheatOn();
void SetupADC();
void RestartADC(void);
unsigned char GetCurrentVoltage();
void setInitialHeight(char newValue);
void setSensetivity(unsigned char s);

#endif /*__ADC_H_*/
