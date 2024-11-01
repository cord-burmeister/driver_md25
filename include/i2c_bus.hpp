// Copyright 2024 Cord Burmeister
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

#ifndef __I2C_BUS_HPP__
#define __I2C_BUS_HPP__

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

/// @brief The class is the controller for a I2C bus with several peripherals 
class I2cBus 
{
public:
  I2cBus(const char * _i2c_file = "/dev/i2c-1");
  ~I2cBus();

  //-------------------- 
  uint8_t readByte(const rclcpp::Logger logger, int deviceId, uint8_t reg, bool & success);
  std::pair<uint8_t, uint8_t> readTwoBytes(const rclcpp::Logger logger, int deviceId, uint8_t reg, bool & success);
  std::pair<int, int> readTwoIntFrom8Bytes(const rclcpp::Logger logger, int deviceId, uint8_t reg, bool & success);
  bool sendCommand(const rclcpp::Logger logger, int deviceId, uint8_t command,uint8_t reg);
  bool selectDevice(const rclcpp::Logger logger, int deviceId);
  std::vector<uint8_t> readBytesFromBus(const rclcpp::Logger logger, int deviceId, uint8_t reg, int count, bool & success);
  std::vector<int> readIntsFromBus(const rclcpp::Logger logger, int deviceId, uint8_t reg, int count, bool & success);
  bool writeIntsToBus(const rclcpp::Logger logger, int deviceId, uint8_t reg, std::vector<int> values);
  bool writeBytesToBus(const rclcpp::Logger logger, int deviceId, uint8_t reg, std::vector<uint8_t> values);

  //-------------
private:
  std::mutex lock;
  int m_fd = -1;
  int lastDeviceId = -1;
  const char * m_i2c_file      = nullptr;

  };

#endif
