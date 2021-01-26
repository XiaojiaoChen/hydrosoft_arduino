/*
 * PRESSURESENSOR.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Xiaojiao Chen
 *		 Email: chen2014@connect.hku.hk
 *	Laboratory: Bio-Control Lab
 * Orgnization: the University of Hong Kong
 */

#include <Arduino.h>
#include "PRESSURESENSOR.h"


//gauge pressure Pa
const int32_t PascalPerPSI=6895;
const int32_t pressureSensorType1_pMin =   0;
const int32_t pressureSensorType1_pMax =  50*PascalPerPSI;


PRESSURE_SENSOR::PRESSURE_SENSOR(int num):
pressure(0)
{
	attach(num);
	setSensorRange_GaugePa(pressureSensorType1_pMin,pressureSensorType1_pMax);
}

int32_t PRESSURE_SENSOR::read()
{
	pressure = map(analogRead(AnalogPort),0,1024,pMin,pMax);
	return pressure;
}

void PRESSURE_SENSOR::attach(int num)
{
	AnalogPort = num;

}

void PRESSURE_SENSOR::setSensorRange_GaugePa(int32_t pmin,int32_t pmax)
{
	pMin=pmin;
	pMax=pmax;
}
