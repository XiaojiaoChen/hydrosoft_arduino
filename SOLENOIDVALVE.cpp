/*
 * SOLENOIDVALVE.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Xiaojiao Chen
 *		 Email: chen2014@connect.hku.hk
 *	Laboratory: Bio-Control Lab
 * Orgnization: the University of Hong Kong
 */

#include <Arduino.h>
#include "SOLENOIDVALVE.h"


SOLENOID_VALVE::SOLENOID_VALVE(uint8_t num){

	_duty=0;
	duty=(PWM_DIGITAL_OUTPUT_REVERSED == 0)?_duty:(1-_duty);
	dutyMin = 0;
	dutyMax = 1;
	attach(num);
}

void SOLENOID_VALVE::attach(uint8_t num)
{
	PWMPort = num;
}

void SOLENOID_VALVE::writeDuty(int16_t dut)
{
	 duty = dut;
	_duty= (PWM_DIGITAL_OUTPUT_REVERSED == 0)?duty:(1-duty);

	if(_duty>0)
		digitalWrite(PWMPort,HIGH);
	else
		digitalWrite(PWMPort,LOW);

}
