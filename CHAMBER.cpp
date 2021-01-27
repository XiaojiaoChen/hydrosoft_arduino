/*
 * CHAMBER.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Xiaojiao Chen
 *		 Email: chen2014@connect.hku.hk
 *	Laboratory: Bio-Control Lab
 * Orgnization: the University of Hong Kong
 */
#include <Arduino.h>
#include "CHAMBER.h"

const int32_t CHAMBER::pressureMin= -100000;
const int32_t CHAMBER::pressureMax=350000;


CHAMBER::CHAMBER(int PWMPort1,int PWMPort2,int PressurePort):
valves{PWMPort1,PWMPort2},
pressureSensor(PressurePort)
{
	pressure=0;
	pressureRaw=0;
	pressureCommand=pressure;
	pressureDeadZone = 2000;
	opening = 0;
	pressureOffset=0;
}


void CHAMBER::attach(int PWMPort1,int PWMPort2,int AnalogPort)
{
	valves[0].attach(PWMPort1);
	valves[1].attach(PWMPort2);
	pressureSensor.attach(AnalogPort);
}


void CHAMBER::attachPWM(int PWMPort1,int PWMPort2)
{
	valves[0].attach(PWMPort1);
	valves[1].attach(PWMPort2);
}
void CHAMBER::attachSensor(int AnalogPort)
{
	pressureSensor.attach(AnalogPort);
}
void CHAMBER::zeroPressure(){
	pressureOffset = pressureRaw;
}

int32_t CHAMBER::readPressure(){
	pressureRaw = pressureSensor.read();
	pressure=pressureRaw-pressureOffset;
	return pressure;
}

void CHAMBER::writePressure(int32_t pNom)//Pa
{
	pressureCommand=pNom;
	int32_t pErr = pressureCommand-pressure;
	if(pErr>pressureDeadZone)
	{
		opening=1;
	}
	else if(pErr<-pressureDeadZone)
	{
		opening=-1;
	}
	else
	{
		opening=0;
	}

	writeOpening(opening);
}
void CHAMBER::writeOpening(int16_t op){
	opening=op;
	if(op==0){
		valves[0].writeDuty(0);
		valves[1].writeDuty(0);
	}
	else if(op==2){
		valves[0].writeDuty(1);
		valves[1].writeDuty(1);
	}
	else if(op>0){

		valves[0].writeDuty(opening);
		valves[1].writeDuty(0);
	}
	else
	{
		valves[0].writeDuty(0);
		valves[1].writeDuty(-opening);
	}

}
