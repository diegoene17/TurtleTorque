/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#ifndef TURTLETORQUE3_MOTOR_DRIVER_H_
#define TURTLETORQUE3_MOTOR_DRIVER_H_

#include "variant.h"
#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_CURRENT             102
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_CURRENT          126
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T and XM430-W350-T)
#define WAFFLE_DXL_LIMIT_MAX_CURRENT             1193

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_POSITION             4
#define LEN_X_GOAL_CURRENT              2
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_CURRENT           2
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define DXL_LEFT_ID                     1       // ID of left motor
#define DXL_RIGHT_ID                    2       // ID of right motor

#define BAUDRATE                        1000000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define LEFT                            0
#define RIGHT                           1

//Aqui ira nuestra constante de torque/corriente una vez calculada
#define VELOCITY_CONSTANT_VALUE         41.69988758  // V = r * w = r     *        (RPM             * 0.10472)
                                                     //           = r     * (0.229 * Goal_Velocity) * 0.10472
                                                     //
                                                     // Goal_Velocity = V / r * 41.69988757710309

#define DEBUG_SERIAL  SerialBT2

class TurtleTorque3MotorDriver
{
 public:
  TurtleTorque3MotorDriver();
  ~TurtleTorque3MotorDriver();
  bool init(String turtlebot3);
  void close(void);
  bool setTorque(bool onoff);
  bool getTorque();
  bool readEncoder(int32_t &left_value, int32_t &right_value);
  bool readCurrent(int32_t &left_value, int32_t &right_value);
  bool writeVelocity(int64_t left_value, int64_t right_value);
  bool writeCurrent(int64_t left_value, int64_t right_value);                   //No sabemos si es int64 o int32
  bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t left_wheel_id_;
  uint8_t right_wheel_id_;
  bool torque_;

  uint16_t dynamixel_limit_max_velocity_;
  uint16_t dynamixel_limit_max_current_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteCurrent_;
  dynamixel::GroupSyncRead *groupSyncReadEncoder_;
  dynamixel::GroupSyncRead *groupSyncReadCurrent_;
};

#endif // TURTLEBOT3_MOTOR_DRIVER_H_
