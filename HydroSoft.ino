
#include <inttypes.h>
#include <ros.h>
#include <hydrosoft_ros/Command_Arm.h>
#include <hydrosoft_ros/Sensor_Arm.h>
#include <hydrosoft_ros/Command_Position_Arm.h>
#include <std_msgs/String.h>
#include "SoftArm.h"
#include <sensor_msgs/JointState.h>
#define ACTALLNUM 8

hydrosoft_ros::Command_Position_Arm posCmd;
//Port configuration on mega2560
int valveControlPorts[16] =
    // Act1      Act2       Act3      Act4
    //In,Out,   In,Out,   In,Out,   In,Out,
    {
        2, 3, 4, 5, 6, 7, 8, 9, //Seg 1  (Root segment)
        2, 3, 4, 5, 6, 7, 8, 9, //Seg 2
};
int PumpControlPorts[2] =
    { // PumpHigh      PumpLow
        1, 2};

int PumpSensorPorts[2] = {
    // PumpHigh      PumpLow
    3, 4};

int32_t pressureUp = 30000;    //30,000Pa limit  for positive pressure
int32_t pressureDown = -30000; //-30,000Pa limit for negative pressure

/*********** in Pa ******************/

class CommandBufferType{
public:
  int16_t actuatorBuffer[ACTALLNUM];
  int16_t pumpInBuffer;
  int16_t pumpOutBuffer;
};
CommandBufferType commandBuffer;

class SensorBufferType{
public:
  int16_t actuatorBuffer[ACTALLNUM];
  int16_t pumpInBuffer;
  int16_t pumpOutBuffer;
};
SensorBufferType sensorBuffer;


int32_t pressureActuator[ACTALLNUM];

int aa=1;
/*********** transfer in KPa, otherwise all in Pa ******************/
hydrosoft_ros::Command_Arm command_msg; //transfer in KPa
hydrosoft_ros::Sensor_Arm sensor_msg;   //transfer in KPa

void subscriberCallback(const hydrosoft_ros::Command_Arm &cmd_msg)
{

  for (int i = 0; i < ACTALLNUM; i++)
  {
      pressureActuator[i] = ((int32_t)cmd_msg.actuator[i]) * 1000;
  }
  pressureUp = command_msg.pumpIn * 1000;
  pressureDown = command_msg.pumpOut * 1000;
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
  sensor_msg.actuator=sensorBuffer.actuatorBuffer;
  sensor_msg.actuator_length=ACTALLNUM;
  command_msg.actuator=commandBuffer.actuatorBuffer;
  command_msg.actuator_length=ACTALLNUM;

}

/***********Only transfer in KPa, otherwise all in Pa ******************/
void loop()
{
  //Serial.println("Arduino Looping");

  /*Maintain the up and down pressure from Host command*/
  softArm.maintainUpPressure(pressureUp, pressureUp + 10000);
  softArm.maintainDownPressure(pressureDown - 10000, pressureDown);

  /*If actuators have sensors*/
  //softArm.readPressureAll();

  /*control actuator pressure from Host command*/
  softArm.writePressureAll(pressureActuator);

  /*Filling the message, in KPa*/
  sensorBuffer.pumpInBuffer=  (int16_t)(softArm.pSource.pressure / 1000);
  sensorBuffer.pumpOutBuffer= (int16_t)(softArm.pSink.pressure / 1000);

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      sensorBuffer.actuatorBuffer[4*i+j] = (int16_t)(softArm.actuators[i][j].opening);
      // Serial.print(i);
      // Serial.print(' ');
      // Serial.print(j);
      // Serial.print(' ');
      // Serial.println(sensorBuffer.actuatorBuffer[4*i+j]);
    }
  }
  
  pub_.publish(&sensor_msg);

  node_handle.spinOnce();
  delay(1);
}
