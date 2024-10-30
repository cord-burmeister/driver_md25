// Copyright 2017 Hunter L. Allen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __MD25_HPP__
#define __MD25_HPP__

#include <functional>
#include <iostream>
#include <cstring>
#include <memory>
#include <tuple>
#include <mutex>
#include <linux/i2c-dev.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <unistd.h>
#include <fcntl.h>

#include <rclcpp/rclcpp.hpp>

#include <i2c_bus.hpp>
#include <md25_driver/motor_controller.hpp>

#define BUF_LEN 10

class md25_driver : public MotorController
{
public:
  md25_driver();
  ~md25_driver();

   bool setup(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus) override  { 
      uint8_t m_software_version_front = 0;
      uint8_t m_software_version_rear = 0;
      bool success = true;
      bool result = true;

      m_software_version_front = i2c_bus->readByte (logger, this->getDeviceIdFront(), SW_VER, success);
      if (!success)
      {
        result = false;
        RCLCPP_ERROR(logger, "MD25 Motors initialized front device failed");
      }
      m_software_version_rear = i2c_bus->readByte (logger, this->getDeviceIdRear(), SW_VER, success);
      if (!success)
      {
        result = false;
        RCLCPP_ERROR(logger, "MD25 Motors initialized rear device failed");
      }
      RCLCPP_INFO(logger, "MD25 Motors initialized with software version '%u' and '%u'", m_software_version_front, m_software_version_rear);

      success = i2c_bus->sendCommand (logger, getDeviceIdFront(), motor_mode_ ,MODE);
      if (!success)
      {
        result = false;
        RCLCPP_ERROR(logger, "MD25 Motors setting mode front device failed");
      }
      success = i2c_bus->sendCommand (logger, getDeviceIdRear(), motor_mode_ ,MODE);
      if (!success)
      {
        result = false;
        RCLCPP_ERROR(logger, "MD25 Motors setting mode rear device failed");
      }
      success = i2c_bus->sendCommand (logger, getDeviceIdFront(), acceleration_rate ,ACC_RATE);
      if (!success)
      {
        result = false;
        RCLCPP_ERROR(logger, "MD25 Motors setting acceleration mode front device failed");
      }
      success = i2c_bus->sendCommand (logger, getDeviceIdRear(), acceleration_rate,ACC_RATE);
      if (!success)
      {
        result = false;
        RCLCPP_ERROR(logger, "MD25 Motors setting acceleration mode rear device failed");
      }
      return result;
  }
 

  bool resetEncoders(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus) override {
    bool success = true;
    bool result = i2c_bus->sendCommand(logger, getDeviceIdFront(), ENCODER_RESET,CMD);
    if (!result) {
      RCLCPP_ERROR(logger, "resetEncoders: Could not reset front encoders");
      success = false;
    }
    result = i2c_bus->sendCommand(logger, getDeviceIdRear(), ENCODER_RESET,CMD);
    if (!result) {
      RCLCPP_ERROR(logger, "resetEncoders: Could not reset rear encoders");
      success = false;
    }
    m_encoder_1_ticks = 0;
    m_encoder_2_ticks = 0;
    m_encoder_3_ticks = 0;
    m_encoder_4_ticks = 0;

    return success;
  }

  std::vector<int> getMotorsSpeed(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, bool & success) override {
    (void) logger; // Swallow unused warning
    (void) i2c_bus; // Swallow unused warning
    int speed;
    std::vector<int> speeds;
    speed = i2c_bus->readByte (logger, this->getDeviceIdFront(), SPD1, success);
    if (!success)
    {
      RCLCPP_ERROR(logger, "MD25 Motors initialized rear device failed");
      return speeds;
    }
    speeds.push_back (speed);
    speed = i2c_bus->readByte (logger, this->getDeviceIdFront(), SPD2, success);
    if (!success)
    {
      RCLCPP_ERROR(logger, "MD25 Motors initialized rear device failed");
      return speeds;
    }
    speeds.push_back (speed);
    speed = i2c_bus->readByte (logger, this->getDeviceIdRear(), SPD2, success);
    if (!success)
    {
      RCLCPP_ERROR(logger, "MD25 Motors initialized rear device failed");
      return speeds;
    }
    speeds.push_back (speed);
    speed = i2c_bus->readByte (logger, this->getDeviceIdRear(), SPD1, success);
    if (!success)
    {
      RCLCPP_ERROR(logger, "MD25 Motors initialized rear device failed");
      return speeds;
    }
    speeds.push_back (speed);
    return speeds;
  }

  std::vector<int> readEncoders(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, bool & success) override {
    std::vector<int> encoderValues;
    int ticks_l;
    int ticks_r;
    std::tie(ticks_l, ticks_r) = i2c_bus->readTwoIntFrom8Bytes (logger, this->getDeviceIdFront(), ENC1, success);
    encoderValues.push_back (ticks_l);
    encoderValues.push_back (ticks_r);
    std::tie(ticks_l, ticks_r) = i2c_bus->readTwoIntFrom8Bytes (logger, this->getDeviceIdRear(), ENC1, success);
    encoderValues.push_back (ticks_l);
    encoderValues.push_back (ticks_r);
    return encoderValues;
  }

  virtual bool stopMotors(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus){
   bool success = true;
    bool result = i2c_bus->sendCommand(logger, getDeviceIdFront(), ENCODER_RESET,CMD);
    if (!result) {
      RCLCPP_ERROR(logger, "resetEncoders: Could not reset front encoders");
      success = false;
    }
    result = i2c_bus->sendCommand(logger, getDeviceIdRear(), ENCODER_RESET,CMD);
    if (!result) {
      RCLCPP_ERROR(logger, "resetEncoders: Could not reset rear encoders");
      success = false;
    }
    return success;
  }
  //-------------------- 
  int getSoftwareVersion(const rclcpp::Logger logger, int deviceId);
  int getBatteryVolts(const rclcpp::Logger logger, int deviceId);
  int getAccelerationRate(const rclcpp::Logger logger, int deviceId);
  int getMode(const rclcpp::Logger logger, int deviceId);
  std::pair<int,int> getMotorsSpeed(const rclcpp::Logger logger, int deviceId);
  std::pair<int,int> getMotorsCurrent(const rclcpp::Logger logger, int deviceId);
  bool enableSpeedRegulation(const rclcpp::Logger logger, int deviceId);
  bool disableSpeedRegulation(const rclcpp::Logger logger, int deviceId);
  bool enableTimeout(const rclcpp::Logger logger, int deviceId);
  bool disableTimeout(const rclcpp::Logger logger, int deviceId);
  bool setMotorsSpeed(const rclcpp::Logger logger, int deviceId, int speed1,int speed2);
  bool stopMotors(const rclcpp::Logger logger, int deviceId);
  bool haltMotors(const rclcpp::Logger logger, int deviceId);
  bool setMode(const rclcpp::Logger logger, int deviceId, int mode);
  bool setAccelerationRate(const rclcpp::Logger logger, int deviceId, int rate);
  std::pair<int, int> readEncoders(const rclcpp::Logger logger, int deviceId);
  bool writeSpeed(const rclcpp::Logger logger, int deviceId, int left,int right);
 
  int getDeviceIdFront ();
  int getDeviceIdRear ();

  int getFrontLeftEncoderId ();
  int getFrontRightEncoderId ();
  int getRearLeftEncoderId ();
  int getRearRightEncoderId ();

  //-------------
private:

  //   Mode Register
  // The mode register selects which mode of operation and I2C data input type the user requires. The options being:
  // 0,    (default setting) If a value of 0 is written to the mode register then the meaning of the speed registers is literal speeds in the range of 0 (full reverse)  128 (stop)   255 (full forward).
  // 1,    Mode 1 is similar to mode 0, except that the speed registers are interpreted as signed values. The meaning of the speed registers is literal speeds in the range of -128 (full reverse)   0 (Stop)   127 (full forward).
  // 2,    Writing a value of  2 to the mode register will make Speed1 control both motors speed, and Speed2 becomes the turn value. 
  // Data is in the range of 0 (full reverse)  128 (stop)  255 (full  forward).
  // 3,    Mode 3 is similar to mode 2, except that the speed registers are interpreted as signed values. 
  // Data is in the range of -128  (full reverse)  0 (stop)   127 (full forward)
  int motor_mode_ = 1;

  // Acceleration Rate 
  // If you require a controlled acceleration period for the attached motors to reach there ultimate speed, the MD25 has a register to provide this. 
  // It works by using a value into the acceleration register and incrementing the power by that value. Changing between the current speed of the motors 
  // and the new speed (from speed 1 and 2 registers). So if the motors were traveling at full speed in the forward direction (255) and were instructed 
  // to move at full speed in reverse (0), there would be 255 steps with an acceleration register value of 1, but 128 for a value of 2. 
  // The default acceleration value is 5, meaning the speed is changed from full forward to full reverse in 1.25 seconds. The register will accept values 
  // of 1 up to 10 which equates to a period of only 0.65 seconds to travel from full speed in one direction to full speed in the opposite direction.
  // See https://www.robot-electronics.co.uk/htm/md25i2c.htm
  int acceleration_rate = 3;

  int deviceIdFront = 0x58;
  int deviceIdRear = 0x5A;

  int m_software_version_front = 0;
  int m_software_version_rear = 0;
  long m_encoder_1_ticks = 0;
  long m_encoder_2_ticks = 0;
  long m_encoder_3_ticks = 0;
  long m_encoder_4_ticks = 0;

  rclcpp::Time last_time;
  
  static const int SPD1		            = 0x00;  // speed to first motor
  static const int SPD2		            = 0x01;  // speed to second motor
  static const int ENC1	              = 0x02;  // motor encoder 1 (first byte)
  static const int ENC2	              = 0x06;  // motor encoder 2 (first byte)
  static const int VOLT		            = 0x0A;  // battery volts
  static const int I1	                = 0x0B;  // motor 1 current
  static const int I2	                = 0x0C;  // motor 2 current
  static const int SW_VER             = 0x0D;  // software version
  static const int ACC_RATE	          = 0x0E;  // acceleration rate
  static const int MODE		            = 0x0F;  // mode of operation
  static const int CMD		            = 0x10;  // command register
  static const int ENCODER_RESET      = 0x20; // 
  static const int DISABLE_SPEED_REG  = 0x30; //
  static const int ENABLE_SPEED_REG   = 0x31; //
  static const int DISABLE_TIMEOUT    = 0x32; //  
  static const int ENABLE_TIMEOUT     = 0x33; //
  static const int STOP_SPEED	      	= 0x00;  // 0 velocity  0x80 = 128


  };

#endif
