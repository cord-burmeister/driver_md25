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

#ifndef __MOTOR_CONTROLLER_HPP__
#define __MOTOR_CONTROLLER_HPP__

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

#define BUF_LEN 10

class MotorController 
{
public:

  //-------------------- 
  virtual bool setup(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus) { 
    (void) logger; // Swallow unused warning
    (void) i2c_bus; // Swallow unused warning
    return true;
  }

  virtual bool resetEncoders(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus) {
    (void) logger; // Swallow unused warning
    (void) i2c_bus; // Swallow unused warning
    return true;
  }

  virtual std::vector<uint8_t> getMotorsSpeed(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, bool & success){
    (void) logger; // Swallow unused warning
    (void) i2c_bus; // Swallow unused warning
    success = true;
    std::vector<uint8_t> speeds;
    return speeds;
  }

 virtual std::vector<int> readEncoders(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, bool & success) {
    (void) logger; // Swallow unused warning
    (void) i2c_bus; // Swallow unused warning
    success = true;
    std::vector<int> encoderValues;
    return encoderValues;
  }

  virtual bool stopMotors(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus){
    (void) logger; // Swallow unused warning
    (void) i2c_bus; // Swallow unused warning
    return true;
  }

  virtual bool setMotorsSpeed(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int frontLeftSpeed, int frontRightSpeed, int rearLeftSpeed, int rearRightSpeed) {
    (void) logger; // Swallow unused warning
    (void) i2c_bus; // Swallow unused warning
    (void) frontLeftSpeed; // Swallow unused warning
    (void) frontRightSpeed; // Swallow unused warning
    (void) rearLeftSpeed; // Swallow unused warning
    (void) rearRightSpeed; // Swallow unused warning
    return true;
  }

  // virtual int getSoftwareVersion(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual int getBatteryVolts(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual int getAccelerationRate(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual int getMode(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual std::pair<int,int> getMotorsSpeed(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual std::pair<int,int> getMotorsCurrent(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual bool enableSpeedRegulation(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual bool disableSpeedRegulation(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual bool enableTimeout(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual bool disableTimeout(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual bool stopMotors(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual bool haltMotors(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual bool setMode(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId, int mode);
  // virtual bool setAccelerationRate(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId, int rate);
  // virtual std::pair<int, int> readEncoders(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId);
  // virtual bool writeSpeed(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int deviceId, int left,int right);

  virtual ~MotorController() = default; // Virtual destructor
  };

#endif
