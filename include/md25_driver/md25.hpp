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


#define BUF_LEN 10

class md25_driver 
{
public:
  //md25_driver(const rclcpp::Logger logger, const char * _i2c_file = "/dev/i2c-1");
  md25_driver(const char * _i2c_file = "/dev/i2c-1");
  ~md25_driver();

  //-------------------- 
  bool setup(const rclcpp::Logger logger);
  int getSoftwareVersion(const rclcpp::Logger logger);
  int getBatteryVolts(const rclcpp::Logger logger);
  int getAccelerationRate(const rclcpp::Logger logger);
  int getMode(const rclcpp::Logger logger);
  std::pair<int,int> getMotorsSpeed(const rclcpp::Logger logger);
  std::pair<int,int> getMotorsCurrent(const rclcpp::Logger logger);
  bool enableSpeedRegulation(const rclcpp::Logger logger);
  bool disableSpeedRegulation(const rclcpp::Logger logger);
  bool enableTimeout(const rclcpp::Logger logger);
  bool disableTimeout(const rclcpp::Logger logger);
  bool setMotorsSpeed(const rclcpp::Logger logger, int speed1,int speed2);
  bool stopMotors(const rclcpp::Logger logger);
  bool haltMotors(const rclcpp::Logger logger);
  bool setMode(const rclcpp::Logger logger, int mode);
  bool setAccelerationRate(const rclcpp::Logger logger, int rate);
  bool resetEncoders(const rclcpp::Logger logger);
  std::pair<int, int> readEncoders(const rclcpp::Logger logger);
  bool writeSpeed(const rclcpp::Logger logger, int left,int right);
 
  //-------------
private:
  std::mutex lock;
  int readByte(const rclcpp::Logger logger, int reg);
  std::pair<int, int> readTwoBytes(const rclcpp::Logger logger, int reg);
  bool sendCommand(const rclcpp::Logger logger, int command,int reg);
  int readEncoder(const rclcpp::Logger logger, int LR);
  bool lastReadEncoders = false;
  int m_fd = -1;
  int address = 0x58;

  int m_software_version = 0;
  long m_encoder_1_ticks = 0;
  long m_encoder_2_ticks = 0;
  const char * m_i2c_file      = nullptr;
  
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
