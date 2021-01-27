
#include <inttypes.h>
#include <ros.h>
#include <hydrosoft_ros/Command_Arm.h>
#include <hydrosoft_ros/Sensor_Arm.h>
#include <hydrosoft_ros/Command_Position_Arm.h>
#include <std_msgs/String.h>
#include "SoftArm.h"
#define ACTALLNUM 8

#define MANNUALMASK (0x0001)
#define PUMPOUTMASK (0x0002)
#define PUMPINMASK (0x0004)
#define VALVEOUTMASK (0x0008)
#define VALVEINMASK (0x0010)

#define __SET_BIT(__VALUE__, __BITMASK__) ((__VALUE__) |= (__BITMASK__))
#define __RESET_BIT(__VALUE__, __BITMASK__) ((__VALUE__) &= ~(__BITMASK__))
#define __TOGGLE_BIT(__VALUE__, __BITMASK__) ((__VALUE__) ^= (__BITMASK__))
#define __GET_BIT(__VALUE__, __BITMASK__) (((__VALUE__) & (__BITMASK__)) != 0)
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

int32_t commandActuator[ACTALLNUM];  //translated command in Pa

int32_t pressureUp=30000;    //30,000Pa limit  for positive pressure
int32_t pressureDown=-30000; //-30,000Pa limit for negative pressure

/*********** in Pa ******************/

class CommandTypeDef
{
public:

  uint16_t mannual_control; //0:auto control   1: mannual control
  uint16_t pumpIn;          //0:Force Off    1:Force On    ,only effecive when mannual_control==1;
  uint16_t pumpOut;         //0:Force Off    1:Force On  ,only effecive when mannual_control==1;
  uint16_t valveIn;         //0:Force Close    1:Force Open  ,only effecive when mannual_control==1;
  uint16_t valveOut;        //0:Force Close    1:Force Open  ,only effecive when mannual_control==1;
};
CommandTypeDef host_command;

int16_t sensorActuatorBuffer[ACTALLNUM];  //sensor buffer in KPa to store the sensor message





hydrosoft_ros::Sensor_Arm sensor_msg; //transfer in KPa

void subscriberCallback(const hydrosoft_ros::Command_Arm &cmd_msg)
{

  for (int i = 0; i < ACTALLNUM; i++)
  {
    commandActuator[i] = ((int32_t)cmd_msg.actuator[i]) * 1000;
  }
  pressureUp = ((int32_t)cmd_msg.pumpIn) * 1000;    //transfer in KPa, local in Pa
  pressureDown = ((int32_t)cmd_msg.pumpOut) * 1000; //transfer in KPa, local In Pa
  host_command.mannual_control = __GET_BIT(cmd_msg.cmd, MANNUALMASK);
  host_command.pumpIn = __GET_BIT(cmd_msg.cmd, PUMPINMASK);
  host_command.pumpOut = __GET_BIT(cmd_msg.cmd, PUMPOUTMASK);
  host_command.valveIn = __GET_BIT(cmd_msg.cmd, VALVEINMASK);
  host_command.valveOut = __GET_BIT(cmd_msg.cmd, VALVEOUTMASK);
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
  softArm.setupPumpPorts(PumpControlPorts[0], PumpSensorPorts[0],PumpControlPorts[1], PumpSensorPorts[1]);

  /*setup buffer pointer and length*/
  sensor_msg.actuator = sensorActuatorBuffer;
  sensor_msg.actuator_length = ACTALLNUM;



}

/***********Only transfer in KPa, otherwise all in Pa ******************/
void loop()
{
  //automatic switch. Press g to start and press s to stop.
  if (!host_command.mannual_control)
  {
    /*Maintain the up and down pressure from Host command*/
    softArm.maintainUpPressure(pressureUp, pressureUp + 10000);
    softArm.maintainDownPressure(pressureDown - 10000, pressureDown);

    /*If actuators have sensors*/
    //softArm.readPressureAll();

    softArm.writePressureAll(commandActuator);
  }
  else
  {
    if (host_command.pumpIn)
    {
      softArm.startPumpIn();
    }
    else
    {
      softArm.stopPumpIn();
    }

    if (host_command.pumpOut)
    {
      softArm.startPumpOut();
    }
    else
    {
      softArm.stopPumpOut();
    }

    //
    if ((!host_command.valveIn) && (!host_command.valveOut))
    {
      softArm.writeOpeningAll(0);
    }

    else if  ((!host_command.valveIn) && (host_command.valveOut))
    {
      softArm.writeOpeningAll(-1);
    }
    else if  ((host_command.valveIn) && (!host_command.valveOut))
    {
      softArm.writeOpeningAll(1);
    }
    else if  ((host_command.valveIn) && (host_command.valveOut))
    {
      softArm.writeOpeningAll(2);
    }
  }

  /*Filling the pump message, in KPa*/
  sensor_msg.pumpIn = softArm.pSource.pump.status;
  sensor_msg.pumpOut = softArm.pSink.pump.status;

  /*Filling the valve message, in [-1, 0, 1]*/
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      sensor_msg.actuator[4 * i + j] = (int16_t)(softArm.actuators[i][j].opening);
    }
  }
  pub_.publish(&sensor_msg);

  node_handle.spinOnce();
  delay(1);
}
