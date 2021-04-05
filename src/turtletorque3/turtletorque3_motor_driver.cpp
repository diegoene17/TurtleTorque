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

#include "../../include/turtletorque3/turtletorque3_motor_driver.h"

TurtleTorque3MotorDriver::TurtleTorque3MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  left_wheel_id_(DXL_LEFT_ID),
  right_wheel_id_(DXL_RIGHT_ID)
{
  torque_ = false;
  dynamixel_limit_max_current_ = WAFFLE_DXL_LIMIT_MAX_CURRENT;
}

TurtleTorque3MotorDriver::~TurtleTorque3MotorDriver()
{
  close();
}

bool TurtleTorque3MotorDriver::init(String turtlebot3)
{
  DEBUG_SERIAL.begin(57600);
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
    return false;
  }

  // Enable Dynamixel Torque
  setTorque(true);

  groupSyncWriteCurrent_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_CURRENT, LEN_X_GOAL_CURRENT);
  groupSyncCurrentLimit_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, CURRENT_LIMIT, LEN_X_CURRENT_LIMIT);
  groupSyncReadEncoder_  = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  groupSyncReadCurrent_  = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_CURRENT, LEN_X_PRESENT_CURRENT);
  groupSyncReadPWM_      = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_PWM, LEN_X_PRESENT_PWM);

  if (turtlebot3 == "Burger"){
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
  } else if (turtlebot3 == "Waffle or Waffle Pi") {
    dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;
    dynamixel_limit_max_current_ = WAFFLE_DXL_LIMIT_MAX_CURRENT;
  } else {
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
  }

  DEBUG_SERIAL.println("Success to init Motor Driver");
  return true;
}

bool TurtleTorque3MotorDriver::setTorque(bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  torque_ = onoff;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_LEFT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_RIGHT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  return true;
}

bool TurtleTorque3MotorDriver::getTorque()
{
  return torque_;
}

void TurtleTorque3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  portHandler_->closePort();
  DEBUG_SERIAL.end();
}

bool TurtleTorque3MotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  left_value  = groupSyncReadEncoder_->getData(left_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_value = groupSyncReadEncoder_->getData(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  groupSyncReadEncoder_->clearParam();
  return true;
}

bool TurtleTorque3MotorDriver::readPWM(uint16_t &left_value, uint16_t &right_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  uint8_t dxl_error = 0;                           //Communication error

  // Read dynamixel1 present pwm
  dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_,left_wheel_id_,ADDR_X_PRESENT_PWM,&left_value,&dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }
  // Read dynamixel2 present pwm
  dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_,right_wheel_id_,ADDR_X_PRESENT_PWM,&right_value,&dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }
  return true;
}

bool TurtleTorque3MotorDriver::readCurrent(uint16_t &left_value, uint16_t &right_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  uint8_t dxl_error = 0;                           //Communication error

  // Read dynamixel1 present current
  dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_,left_wheel_id_,ADDR_X_PRESENT_CURRENT,&left_value,&dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }
  // Read dynamixel2 present current
  dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_,right_wheel_id_,ADDR_X_PRESENT_CURRENT,&right_value,&dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }
  return true;
}

bool TurtleTorque3MotorDriver::writeCurrent(int left_value, int right_value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_t left_data_byte[2];
  uint8_t right_data_byte[2];


  left_data_byte[0] = DXL_LOBYTE(left_value);
  left_data_byte[1] = DXL_HIBYTE(left_value);

  dxl_addparam_result = groupSyncWriteCurrent_->addParam(left_wheel_id_, left_data_byte);
  //dxl_addparam_result = groupSyncCurrentLimit_->addParam(left_wheel_id_, (uint8_t*)&left_data_byte);

  if (dxl_addparam_result != true){
    //Borrar impresión cuando comprobemos
    DEBUG_SERIAL.println("Fallo motor izquierdo");
    return false;
  }

  right_data_byte[0] = DXL_LOBYTE(right_value);
  right_data_byte[1] = DXL_HIBYTE(right_value);

  dxl_addparam_result = groupSyncWriteCurrent_->addParam(right_wheel_id_, right_data_byte);
  //dxl_addparam_result = groupSyncCurrentLimit_->addParam(left_wheel_id_, (uint8_t*)&left_data_byte);
  if (dxl_addparam_result != true){
    //Borrar impresión cuando comprobemos
    DEBUG_SERIAL.println("Fallo motor derecho");
    return false;
  }

  dxl_comm_result = groupSyncWriteCurrent_->txPacket();
  //dxl_comm_result = groupSyncCurrentLimit_->txPacket();

  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWriteCurrent_->clearParam();
  //groupSyncCurrentLimit_->clearParam();
  return true;
}

bool Turtlebot3MotorDriver::writeVelocity(int64_t left_value, int64_t right_value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_t left_data_byte[4] = {0, };
  uint8_t right_data_byte[4] = {0, };


  left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_value));
  left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_value));
  left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_value));
  left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_wheel_id_, (uint8_t*)&left_data_byte);
  if (dxl_addparam_result != true)
    return false;

  right_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_value));
  right_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_value));
  right_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_value));
  right_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_wheel_id_, (uint8_t*)&right_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}


bool TurtleTorque3MotorDriver::controlMotor(int left_value, int right_value)
{
  bool dxl_comm_result = false;
  //Aqui iria el controlador en caso de ser necesario

  dxl_comm_result = writeCurrent(left_value, right_value);
  if(dxl_comm_result == false)
    return false;
  return true;
}
