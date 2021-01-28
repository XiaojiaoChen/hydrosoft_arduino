/*
 *
 *  Created on: May 12, 2018
 *      Author: 402072495
 */

#ifndef USERINC_SOFTARM_H_
#define USERINC_SOFTARM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "CHAMBER.h"
#include "PRESSURESOURCE.h"
#define SEGNUM 2
#define ACTNUM 4
#define P_ATM 101000

	class SOFT_ARM
	{
	public:
		SOFT_ARM();

		CHAMBER actuators[SEGNUM][ACTNUM];

		PRESSURE_SOURCE pSource;
		PRESSURE_SOURCE pSink;

		void setupValvePorts(uint8_t *ports);
		void setupPumpPorts(uint8_t cp1,uint8_t sp1, uint8_t cp2, uint8_t sp2);

		void maintainUpPressure(int32_t posP, int32_t posP2);
		void maintainDownPressure(int32_t negP, int32_t negP2);

		void readPressureAll();
		void writePressureAll(int32_t *pCommand);

		void writeOpening(int16_t op);
		void writeOpeningAll(int16_t *op);
		void zeroPressureAll();
		void execInfoCommand(char *);

		void openValveIn();
		void closeValveIn();
		void openValveOut();
		void closeValveOut();
		void startPumpIn();
		void stopPumpIn();
		void startPumpOut();
		void stopPumpOut();
	};

#ifdef __cplusplus
}
#endif
#endif /* USERINC_SOFTARM_H_ */
