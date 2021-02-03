/*
 * 
 *  Created on: Jun 15, 2018
 *      Author: Xiaojiao Chen
 *		 Email: chen2014@connect.hku.hk
 *	Laboratory: Bio-Control Lab
 * Orgnization: the University of Hong Kong
 */

#ifndef PRESSURESENSOR_H_
#define PRESSURESENSOR_H_
#ifdef __cplusplus
extern "C" {
#endif


#include  <inttypes.h>

class PRESSURE_SENSOR {
public:
	PRESSURE_SENSOR(uint8_t num = 0);
	void attach(uint8_t num);
	uint8_t AnalogPort;

	int32_t read();
	void setSensorRange_GaugePa(int32_t pmin,int32_t pmax);
	int32_t pressure;
	int32_t pMin;
	int32_t pMax;
};


#ifdef __cplusplus
}
#endif
#endif /* PRESSURESENSOR_H_ */
