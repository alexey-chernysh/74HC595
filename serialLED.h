
#ifndef __SERIAL_LED_H__
#define __SERIAL_LED_H__

void SetupSerialLEDs();
void SetDisplay(unsigned char c0, unsigned char c1, unsigned char c2, unsigned char c3);
void SetDisplayNumeric(unsigned char lead_char, unsigned char num);
void resetParamDisplayCounter();

#endif /*__SERIAL_LED_H_*/
