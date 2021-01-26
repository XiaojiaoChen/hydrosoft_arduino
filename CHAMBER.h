/*
 * CHAMBER.h
 *
 *  Created on: Jun 15, 2018
 *      Author: Xiaojiao Chen
 *		 Email: chen2014@connect.hku.hk
 *	Laboratory: Bio-Control Lab
 * Orgnization: the University of Hong Kong
 */

#ifndef CHAMBER_H_
#define CHAMBER_H_

#ifdef __cplusplus
extern "C" {
#endif



#include "PRESSURESENSOR.h"
#include "SOLENOIDVALVE.h"

class CHAMBER{
public:
	CHAMBER(int PWMPort1=0,int PWMPort2=1,int AnalogPort=0);
	void attach(int PWMPort1,int PWMPort2,int AnalogPort);
	void attachPWM(int PWMPort1,int PWMPort2);
	void attachSensor(int AnalogPort);

	int32_t readPressure();//Pa
	void writePressure(int32_t pressureCommand);
	void writeOpening(int16_t opening);
	void zeroPressure();

	SOLENOID_VALVE valves[2];
	PRESSURE_SENSOR pressureSensor;


	int32_t pressure;
	int32_t pressureRaw;
	int32_t pressureOffset;
	int32_t pressureCommand;
	int32_t pressureDeadZone;

	int16_t opening;//([-1 or 1],    -1:maximum deflate;  0:shut off,  1: maximum inflate)

private:
	static const int32_t pressureMin;
	static const int32_t pressureMax;
};

#ifdef __cplusplus
}
#endif
#endif /* CHAMBER_H_ */
