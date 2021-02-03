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


PRESSURE_SOURCE::PRESSURE_SOURCE(uint8_t DigitalPort, uint8_t valvePort, uint8_t AnalogPort,uint8_t sType) {
	attach(DigitalPort, valvePort, AnalogPort);
	sourceType = sType;
	direction=0;
	pressure=0;
	pressureLower=0;
	pressureUpper=0;
	valve.writeDuty(0);
}


void PRESSURE_SOURCE::attach(uint8_t DigitalPort,uint8_t valvePort, uint8_t AnalogPort) {
	attachPump(DigitalPort);
	attachSensor(AnalogPort);
	attachValve(valvePort);
}

void PRESSURE_SOURCE::attachPump(uint8_t DigitalPort) {
	pump.attach(DigitalPort);
}

void PRESSURE_SOURCE::attachSensor(uint8_t AnalogPort) {
	pressureSensor.attach(AnalogPort);
}
void PRESSURE_SOURCE::attachValve(uint8_t valPort) {
	valve.attach(valPort);
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
void PRESSURE_SOURCE::openValve(){
	valve.writeDuty(1);
}
void PRESSURE_SOURCE::closeValve(){
	valve.writeDuty(0);
}

int32_t PRESSURE_SOURCE::readPressure() {
	int32_t pp=pressureSensor.read();
	if(sourceType==LOW_PRESSURE_SINK){
		pp=-pp;
	}
	pressure = pp;
	return pressure;
}
