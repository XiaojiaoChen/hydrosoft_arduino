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
#include "SOLENOIDVALVE.h"
typedef enum{
	HIGH_PRESSURE_SOURCE = 0,
	LOW_PRESSURE_SINK=1
}PRESSURE_SOURCE_TYPE;

class PRESSURE_SOURCE {
public:
	PRESSURE_SOURCE(uint8_t DigitalPort=0,uint8_t valvePort=1,uint8_t AnalogPort=0,uint8_t Psourcetype=HIGH_PRESSURE_SOURCE);
	void attach(uint8_t DigitalPort,uint8_t valPort,uint8_t AnalogPort);
	void attachPump(uint8_t DigitalPort);
	void attachSensor(uint8_t AnalogPort);
	void attachValve(uint8_t valvePort);
	int32_t readPressure();
	void maintainPressure(int32_t p_low,int32_t p_high);
	void stop(void);
	void start(void);
	void openValve(void);
	void closeValve(void);
	int sourceType;
	PUMP pump;
	SOLENOID_VALVE valve;
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
