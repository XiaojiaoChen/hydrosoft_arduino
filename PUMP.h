/*
 * PUMP.h
 *
 *  Created on: Jun 15, 2018
 *      Author: Xiaojiao Chen
 *		 Email: chen2014@connect.hku.hk
 *	Laboratory: Bio-Control Lab
 * Orgnization: the University of Hong Kong
 */

#ifndef PUMP_H_
#define PUMP_H_
#ifdef __cplusplus
extern "C" {
#endif
#define PWM_DIGITAL_OUTPUT_REVERSED 1
#include  <inttypes.h>
class PUMP {
public:
	PUMP(uint8_t num=0);
	void attach(uint8_t num);
	void start();
	void stop();
	uint8_t DigitalPort;
	int16_t status;
};

#ifdef __cplusplus
}
#endif
#endif /* PUMP_H_ */
