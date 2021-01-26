/*
 * SOLENOIDVALVE.h
 *
 *  Created on: Jun 15, 2018
 *      Author: Xiaojiao Chen
 *		 Email: chen2014@connect.hku.hk
 *	Laboratory: Bio-Control Lab
 * Orgnization: the University of Hong Kong
 */

#ifndef SOLENOIDVALVE_H_
#define SOLENOIDVALVE_H_
#ifdef __cplusplus
extern "C" {
#endif
#include  <inttypes.h>
#define PWM_DIGITAL_OUTPUT_REVERSED 1

class SOLENOID_VALVE {
public:
	SOLENOID_VALVE(int num=0);
	void attach(int num);

	void writeDuty(int16_t dut);



private:
	int PWMPort;
	int16_t duty;    //nominal duty. 0 for close and 1 for open
	int16_t _duty;   //physical PWM duty. 
	int16_t dutyMin;
	int16_t dutyMax;
};

#ifdef __cplusplus
}
#endif
#endif /* SOLENOIDVALVE_H_ */
