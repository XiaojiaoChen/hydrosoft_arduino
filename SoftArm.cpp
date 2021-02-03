/*
 * SoftArm.c
 *
 *  Created on: May 12, 2018
 *      Author: 402072495
 */
#include <Arduino.h>
#include "SoftArm.h"

/*************************SOFT ARM**************************
 *
 ***********************************************************/
SOFT_ARM::SOFT_ARM():
pSource(0,0,HIGH_PRESSURE_SOURCE),
pSink(0,0,LOW_PRESSURE_SINK)
{
}

void SOFT_ARM::setupValvePorts(uint8_t *ports)
{
	/*Every actuator is refered to with two numbers, segNum(0-3) and bellowNum(0-3)*/

	for (int j = 0; j < SEGNUM; j++)
	{
		for (int i = 0; i < ACTNUM; i++)
		{
			pinMode(ports[2*(j*ACTNUM+i)+0],OUTPUT);
			pinMode(ports[2*(j*ACTNUM+i)+1],OUTPUT);
			actuators[j][i].attachPWM(ports[2*(j*ACTNUM+i)+0],ports[2*(j*ACTNUM+i)+1]);
			actuators[j][i].writeOpening(0);
		}
	}
}
void SOFT_ARM::setupGripperPorts(uint8_t cp1,uint8_t cp2)
{
	pinMode(cp1,OUTPUT);
	pinMode(cp2,OUTPUT);
	gripper.attachPWM(cp1,cp2);
	gripper.writeOpening(0);
}
void SOFT_ARM::setupPumpPorts(uint8_t cp1,uint8_t vp1,uint8_t sp1, uint8_t cp2, uint8_t vp2,uint8_t sp2)
{
	pinMode(cp1,OUTPUT);
	pinMode(cp2,OUTPUT);
	pinMode(vp1,OUTPUT);
	pinMode(vp2,OUTPUT);
	pSource.attach(cp1, vp1,sp1);
	pSink.attach(cp2,vp2, sp2);
}

void SOFT_ARM::maintainUpPressure(int32_t posP, int32_t posP2)
{
	pSource.maintainPressure(posP, posP2);
}
void SOFT_ARM::maintainDownPressure(int32_t negP, int32_t negP2)
{
	pSink.maintainPressure(negP, negP2);
}

void SOFT_ARM::readPressureAll()
{
	for (int j = 0; j < SEGNUM; j++)
	{
		for (int i = 0; i < ACTNUM; i++)
		{
			actuators[j][i].readPressure();
		}
	}
}

void SOFT_ARM::zeroPressureAll()
{
	for (int j = 0; j < SEGNUM; j++)
	{
		for (int i = 0; i < ACTNUM; i++)
		{
			actuators[j][i].zeroPressure();
		}
	}
}

void SOFT_ARM::writePressureAll(int32_t *pCommand)
{
	for (int j = 0; j < SEGNUM; j++)
	{
		for (int i = 0; i < ACTNUM; i++)
		{
			actuators[j][i].writePressure(pCommand[j*ACTNUM+i]);
		}
	}
}

void SOFT_ARM::writeOpening(int16_t op)
{

	for (int j = 0; j < SEGNUM; j++)
	{
		for (int i = 0; i < ACTNUM; i++)
		{
			actuators[j][i].writeOpening(op);
		}
	}
}

void SOFT_ARM::writeOpeningAll(int16_t *op)
{

	for (int j = 0; j < SEGNUM; j++)
	{
		for (int i = 0; i < ACTNUM; i++)
		{
			actuators[j][i].writeOpening(op[j*ACTNUM+i]);
		}
	}
}

void SOFT_ARM::startPumpIn()
{
	pSource.start();
}
void SOFT_ARM::stopPumpIn()
{
	pSource.stop();
}
void SOFT_ARM::startPumpOut()
{
	pSink.start();
}
void SOFT_ARM::stopPumpOut()
{
	pSink.stop();
}


void SOFT_ARM::execInfoCommand(char *infoBuf)
{
	if (infoBuf[0] == 'z')
	{
	}
	if (infoBuf[0] == 'o')
	{
	}
}
