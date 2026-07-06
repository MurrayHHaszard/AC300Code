/*
 * pwm.h
 *
 *  Created on: 17/10/2016
 *      Author: Murray
 */

#ifndef AC210_PWM_H_
#define AC210_PWM_H_

void AC210_PWM_Init(void);
void PWM_SetPeriod_us(uint32_t period_us);
void PWM_Enable(int n);
void PWM_Disable(int n);
void PWM_SetPulsePercent(int percent);
void PWM_SetFrequency(uint32_t pwm_freq);

#endif /* AC210_PWM_H_ */
