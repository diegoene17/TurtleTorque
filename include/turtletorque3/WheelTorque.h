#ifndef turtletorque3_msgs_WheelTorque_h
#define turtletorque3_msgs_WheelTorque_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtletorque3_msgs
{

  class WheelTorque : public ros::Msg
  {
    public:
      typedef float _wheel_torque_1_type;
      _wheel_torque_1_type wheel_torque_1;
      typedef float _wheel_torque_2_type;
      _wheel_torque_2_type wheel_torque_2;

    WheelTorque():
      wheel_torque_1(0),
      wheel_torque_2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wheel_torque_1;
      u_wheel_torque_1.real = this->wheel_torque_1;
      *(outbuffer + offset + 0) = (u_wheel_torque_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_torque_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_torque_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_torque_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_torque_1);
      union {
        float real;
        uint32_t base;
      } u_wheel_torque_2;
      u_wheel_torque_2.real = this->wheel_torque_2;
      *(outbuffer + offset + 0) = (u_wheel_torque_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_torque_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_torque_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_torque_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_torque_2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wheel_torque_1;
      u_wheel_torque_1.base = 0;
      u_wheel_torque_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_torque_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_torque_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_torque_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_torque_1 = u_wheel_torque_1.real;
      offset += sizeof(this->wheel_torque_1);
      union {
        float real;
        uint32_t base;
      } u_wheel_torque_2;
      u_wheel_torque_2.base = 0;
      u_wheel_torque_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_torque_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_torque_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_torque_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_torque_2 = u_wheel_torque_2.real;
      offset += sizeof(this->wheel_torque_2);
     return offset;
    }

    const char * getType(){ return "turtletorque3_msgs/WheelTorque"; };
    const char * getMD5(){ return "28d351555540cc84fd0ec12595c5bdaa"; };

  };

}
#endif
