
#include <inttypes.h>
#include <ros.h>
#include <hydrosoft_ros/Command_Arm.h>
#include <hydrosoft_ros/Sensor_Arm.h>
#include <hydrosoft_ros/Command_Position_Arm.h>
#include <std_msgs/String.h>
#include "SoftArm.h"
#define ACTALLNUM 8

#define AUTOMASK (uint16_t)(0x0001 << 15)
#define PUMPOUTMASK (uint16_t)(0x0001 << 14)
#define PUMPINMASK (uint16_t)(0x0001 << 13)
#define VALVEOUTMASK (uint16_t)(0x0001 << 12)
#define VALVEINMASK (uint16_t)(0x0001 << 11)

hydrosoft_ros::Command_Position_Arm posCmd;
//Port configuration on mega2560

int valveControlPorts[16] =
    // Act1      Act2       Act3      Act4
    //In,Out,   In,Out,   In,Out,   In,Out,
    {
        29, 45, 31, 47, 33, 49, 22, 38, //Seg 1  (Root segment)
        24, 40, 26, 42, 28, 44, 30, 46  //Seg 2
};
int PumpControlPorts[2] =
    { // PumpHigh      PumpLow
        25, 41};

int PumpSensorPorts[2] = {
    // PumpHigh      PumpLow
    54, 55}; //A0 A1

int32_t pressureUp = 30000;    //30,000Pa limit  for positive pressure
int32_t pressureDown = -30000; //-30,000Pa limit for negative pressure

/*********** in Pa ******************/

class CommandBufferType
{
public:
  int16_t actuatorBuffer[ACTALLNUM];
  int16_t pumpInBuffer;
  int16_t pumpOutBuffer;
};
CommandBufferType commandBuffer;

class SensorBufferType
{
public:
  int16_t actuatorBuffer[ACTALLNUM];
  int16_t pumpInBuffer;
  int16_t pumpOutBuffer;
};
SensorBufferType sensorBuffer;

uint16_t gogogo = 1;

int32_t pressureActuator[ACTALLNUM];

/*mannul control command from Host (X:stopped.   In/Out:Start)
                        0           1           2        3
        cmdPump:       X, X       In, X        X, Out    In,Out  
        cmdValve:      X, X       In, X        X, Out    In,Out  
*/
uint16_t cmdPump = 0;
uint16_t cmdValve = 0;

hydrosoft_ros::Command_Arm command_msg; //transfer in KPa

hydrosoft_ros::Sensor_Arm sensor_msg; //transfer in KPa

void subscriberCallback(const hydrosoft_ros::Command_Arm &cmd_msg)
{

  for (int i = 0; i < ACTALLNUM; i++)
  {
    pressureActuator[i] = ((int32_t)cmd_msg.actuator[i]) * 1000;
  }
  pressureUp = command_msg.pumpIn * 1000;    //transfer in KPa, local in Pa
  pressureDown = command_msg.pumpOut * 1000; //transfer in KPa, local In Pa

   
   gogogo = (uint16_t)((command_msg.cmd & AUTOMASK)>>15);
   cmdPump = (uint16_t)((command_msg.cmd & (PUMPINMASK | PUMPOUTMASK))>>13);
   cmdValve = (uint16_t)((command_msg.cmd & (VALVEINMASK | VALVEOUTMASK))>>11);
  
}

ros::NodeHandle node_handle;
ros::Publisher pub_("HydroStates", &sensor_msg);
ros::Subscriber<hydrosoft_ros::Command_Arm> sub_("HydroCommands", &subscriberCallback);

SOFT_ARM softArm;
void setup()
{
  //Serial.println("Arduino Started");
  node_handle.initNode();
  node_handle.advertise(pub_);
  node_handle.subscribe(sub_);

  /*soft arm Valves' port mapping*/
  softArm.setupValvePorts(valveControlPorts);

  /*soft arm Pumps' port mapping*/
  softArm.setupPumpPorts(PumpControlPorts, PumpControlPorts);

  /*setup buffer pointer and length*/
  sensor_msg.actuator = sensorBuffer.actuatorBuffer;
  sensor_msg.actuator_length = ACTALLNUM;
  command_msg.actuator = commandBuffer.actuatorBuffer;
  command_msg.actuator_length = ACTALLNUM;
}

/***********Only transfer in KPa, otherwise all in Pa ******************/
void loop()
{
  //automatic switch. Press g to start and press s to stop.
  if (gogogo)
  {
    /*Maintain the up and down pressure from Host command*/
    softArm.maintainUpPressure(pressureUp, pressureUp + 10000);
    softArm.maintainDownPressure(pressureDown - 10000, pressureDown);

    /*If actuators have sensors*/
    //softArm.readPressureAll();

    softArm.writePressureAll(pressureActuator);

  }
  else
  {
    if (cmdPump == 0)
    {
      softArm.stopPumpIn();
      softArm.stopPumpOut();
    }
    else if (cmdPump == 1)
    {
      softArm.startPumpIn();
      softArm.stopPumpOut();
    }

    else if (cmdPump == 2)
    {
      softArm.stopPumpIn();
      softArm.startPumpOut();
    }
    else
    {
      softArm.startPumpIn();
      softArm.startPumpOut();
    }

    if (cmdValve == 0)
    {
      softArm.writeOpeningAll(0);
    }
    else if (cmdValve == 1)
    {
      softArm.writeOpeningAll(1);
    }

    else if (cmdValve == 2)
    {
      softArm.writeOpeningAll(-1);
    }
    else if (cmdValve == 3)
    {
      softArm.writeOpeningAll(2);
    }
  }
  /*Filling the pump message, in KPa*/
  sensorBuffer.pumpInBuffer = (int16_t)(softArm.pSource.pressure / 1000);
  sensorBuffer.pumpOutBuffer = (int16_t)(softArm.pSink.pressure / 1000);

  /*Filling the valve message, in [-1, 0, 1]*/
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      sensorBuffer.actuatorBuffer[4 * i + j] = (int16_t)(softArm.actuators[i][j].opening);
    }
  }
   sensorBuffer.actuatorBuffer[0]=command_msg.cmd;
   sensorBuffer.actuatorBuffer[1]=cmdPump;
   sensorBuffer.actuatorBuffer[2]=cmdValve;

  pub_.publish(&sensor_msg);

  node_handle.spinOnce();
  delay(1);
}
