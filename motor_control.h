
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

void SetupMotorControl();
void SetMotorVelocity(signed int velocity); 
void SetPWMLimit(unsigned int pwm_limit);

#endif /*__MOTOR_CONTROL_H_*/

