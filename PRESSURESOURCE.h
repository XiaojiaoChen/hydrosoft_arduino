/*
 * PRESSURE_SOURCE.h
 *
 *  Created on: Jan 15, 2019
 *      Author: Xiaojiao Chen
 *		 Email: chen2014@connect.hku.hk
 *	Laboratory: Bio-Control Lab
 * Orgnization: the University of Hong Kong
 */

#ifndef FRAMEWORK_PNEUMATICELEMENTS_PRESSURESOURCE_H_
#define FRAMEWORK_PNEUMATICELEMENTS_PRESSURESOURCE_H_
#ifdef __cplusplus
extern "C" {
#endif

/*
 *
 */

#include "PUMP.h"
#include "PRESSURESENSOR.h"
class PRESSURE_SOURCE {
public:
	PRESSURE_SOURCE(int DigitalPort=0,int AnalogPort=0);
	void attach(int DigitalPort,int AnalogPort);
	void attachPump(int DigitalPort);
	void attachSensor(int AnalogPort);
	int32_t readPressure();
	void maintainPressure(int32_t p_low,int32_t p_high);
	void stop(void);
	void start(void);
	int sourceType;
	PUMP pump;
	PRESSURE_SENSOR pressureSensor;
	int direction;
	int32_t pressure;
	int32_t pressureLower;
	int32_t pressureUpper;
};

#ifdef __cplusplus
}
#endif
#endif /* FRAMEWORK_PNEUMATICELEMENTS_PRESSURESOURCE_H_ */
