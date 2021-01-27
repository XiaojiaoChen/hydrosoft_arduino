/*
 * PRESSURE_SOURCE.cpp
 *
 *  Created on: Jan 15, 2019
 *      Author: Xiaojiao Chen
 *		 Email: chen2014@connect.hku.hk
 *	Laboratory: Bio-Control Lab
 * Orgnization: the University of Hong Kong
 */

#include "PRESSURESOURCE.h"


PRESSURE_SOURCE::PRESSURE_SOURCE(int DigitalPort, int AnalogPort,int sType) {
	attach(DigitalPort,AnalogPort);
	sourceType = sType;
	direction=0;
	pressure=0;
	pressureLower=0;
	pressureUpper=0;
}


void PRESSURE_SOURCE::attach(int DigitalPort, int AnalogPort) {
	attachPump(DigitalPort);
	attachSensor(AnalogPort);
}

void PRESSURE_SOURCE::attachPump(int DigitalPort) {
	pump.attach(DigitalPort);
}

void PRESSURE_SOURCE::attachSensor(int AnalogPort) {
	pressureSensor.attach(AnalogPort);
}

void PRESSURE_SOURCE::maintainPressure(int32_t p_low,int32_t p_high) {

	pressureLower=p_low;
	pressureUpper=p_high;
	readPressure();
	if(sourceType==LOW_PRESSURE_SINK){
		if(pressure<pressureLower)
		{
			pump.stop();
		}
		else if(pressure>pressureUpper)
		{
			pump.start();
		}
	}
	else if(sourceType==HIGH_PRESSURE_SOURCE){
		if(pressure<pressureLower)
		{
			pump.start();
		}
		else if(pressure>pressureUpper)
		{
			pump.stop();
		}
	}

}
void PRESSURE_SOURCE::stop()
{
	pump.stop();
}
void PRESSURE_SOURCE::start()
{
	pump.start();
}

int32_t PRESSURE_SOURCE::readPressure() {
	int32_t pp=pressureSensor.read();
	if(sourceType==LOW_PRESSURE_SINK){
		pp=-pp;
	}
	pressure = pp;
	return pressure;
}
