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

#ifndef __HIWONDER_HPP__
#define __HIWONDER_HPP__

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
#include <driver/motor_controller.hpp>

#define BUF_LEN 10


/*
    Wiring Diagram 
                      FRONT                        
        +---------------------------------+        
        |           +---------+           |        
        |           |         |           |        
      +---------+   | 0x34    |   +---------+      
      | | 1     |   |         |   |     3 | |      
      | |       |   +--+---+--+   |       | |      
      +---------+      +---+      +---------+      
        |                                 |        
        |                                 |        
LEFT    |                                 |  RIGHT 
        |                                 |        
        |                                 |        
      +--------+                  +---------+      
      | | 2    |                  |     4 | |      
      | |      |                  |       | |      
      +--------+                  +---------+      
        |                                 |        
        +---------------------------------+        
                       REAR                        
                                                   
*/


class hiwonder_driver : public MotorController
{
public:
  hiwonder_driver();
  ~hiwonder_driver();

   bool setup(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus) override  { 
      bool success = true;
      bool result = true;

      success = i2c_bus->sendCommand (logger, getDeviceIdFront(), MOTOR_TYPE_JGB37_520_12V_110RPM ,MOTOR_TYPE_ADDR);
      if (!success)
      {
        result = false;
        RCLCPP_ERROR(logger, "HiWonder Motor Controller setting motor type failed");
      }
      success = i2c_bus->sendCommand (logger, getDeviceIdFront(), 0 ,MOTOR_ENCODER_POLARITY_ADDR);
      if (!success)
      {
        result = false;
        RCLCPP_ERROR(logger, "HiWonder Motor Controller setting motor polarity failed");
      }
      return result;
  }
 

  bool resetEncoders(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus) override {
    bool success = true;
    std::vector<int> nullEncoder = {0, 0, 0, 0};
    
    bool result = i2c_bus->writeIntsToBus(logger, getDeviceIdFront(), MOTOR_ENCODER_TOTAL_ADDR, nullEncoder);
    if (!result) {
      RCLCPP_ERROR(logger, "resetEncoders: Could not reset encoders values");
      success = false;
    }
    return success;
  }

  // Convention of the order of values is 
  // front_left_encoder	= (double) encoderValues[0];			// Last encoder value front left
  // front_right_encoder = (double) encoderValues[1];			// Last encoder value front right 
  // rear_left_encoder	  = (double) encoderValues[2];			// Last encoder value rear left
  // rear_right_encoder	= (double) encoderValues[3];			// Last encoder value rear right 
  std::vector<uint8_t> getMotorsSpeed(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, bool & success) override {
    std::vector<uint8_t> result;
    // Values are sorted by number from the controller
    std::vector<uint8_t> values = i2c_bus->readBytesFromBus (logger, this->getDeviceIdFront(), MOTOR_FIXED_SPEED_ADDR, 4, success);
    if (!success) {
      RCLCPP_ERROR(logger, "getMotorsSpeed: Could not read speed");
      success = false;
    }
    // Arrange values based on convention
    result.push_back(values[0]); // front left
    result.push_back(-values[2]); // front right 
    result.push_back(values[1]); // rear left
    result.push_back(-values[3]); // rear right 
    return result;
  }

  // Convention of the order of values is 
  // front_left_encoder	= (double) encoderValues[0];			// Last encoder value front left
  // front_right_encoder = (double) encoderValues[1];			// Last encoder value front right 
  // rear_left_encoder	  = (double) encoderValues[2];			// Last encoder value rear left
  // rear_right_encoder	= (double) encoderValues[3];			// Last encoder value rear right 
  std::vector<int> readEncoders(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, bool & success) override {
    std::vector<int> result;
    std::vector<int> values = i2c_bus->readIntsFromBus (logger, this->getDeviceIdFront(), MOTOR_ENCODER_TOTAL_ADDR, 4, success);
    // Arrange values based on convention
    result.push_back(-values[0]); // front left
    result.push_back(values[2]); // front right 
    result.push_back(values[1]); // rear left
    result.push_back(-values[3]); // rear right 
    return result;
  }

  bool stopMotors(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus) override {
    bool success = true;
    std::vector<uint8_t> nullSpeed = {0, 0, 0, 0};   
    bool result = i2c_bus->writeBytesToBus (logger, this->getDeviceIdFront(), MOTOR_FIXED_SPEED_ADDR, nullSpeed);
    if (!result) {
      RCLCPP_ERROR(logger, "stopMotors: Could not set speed");
      success = false;
    }
    return success;
  }


  // Unit: pulse count per 10 milliseconds, range: (depending on the specific encoder motor, 
  // affected by the number of encoder wires, voltage, load, etc., generally around ±50))
  bool setMotorsSpeed(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, int frontLeftSpeed, int frontRightSpeed, int rearLeftSpeed, int rearRightSpeed) override {

    bool success = true;
    std::vector<uint8_t> speed;
    // Arrange values based on inverted convention
    // There are 3 effects. 
    // 1. The motor on the tight side have a different physical orientation
    // 2. The connectors for one side of the robot are rotated on the controller which leads to inverted directions.
    // 3. The default rotation direction of the motor. 
    speed.push_back (static_cast<uint8_t>(-frontLeftSpeed));
    speed.push_back (static_cast<uint8_t>(rearLeftSpeed));
    speed.push_back (static_cast<uint8_t>(frontRightSpeed));
    speed.push_back (static_cast<uint8_t>(-rearRightSpeed));   
    bool result = i2c_bus->writeBytesToBus (logger, this->getDeviceIdFront(), MOTOR_FIXED_SPEED_ADDR, speed);
    if (!result) {
      RCLCPP_ERROR(logger, "setMotorsSpeed: Could not set speed");
      success = false;
    }
    return success;
  }


  int getDeviceIdFront ();

  //-------------
private:

  


// Hi wonder documentation based on the TankDemo.py script

  static const int deviceIdFront                = 0x34; // I2C Address

  static const int ADC_BAR_ADDR	      	        = 0x00;  // ADC battery sampling
  static const int MOTOR_TYPE_ADDR	            = 0x14;  // Encoder motor type settings
  static const int MOTOR_ENCODER_POLARITY_ADDR  = 0x15; //  Encoder direction polarity settings 
  static const int MOTOR_FIXED_SPEED_ADDR       = 0x33; // (Fixed speed control, closed-loop control,)，
                                                  // (Unit: pulse count per 10 milliseconds, range: (depending on the specific encoder motor, affected by the number of encoder wires, 
                                                  // voltage, load, etc., generally around ±50))
  static const int MOTOR_ENCODER_TOTAL_ADDR     = 0x3C; // (Total pulse value of each of the four encoder motors)
        // (If the number of pulses per revolution of the motor is known as U, and the diameter of the wheel is known as D, the distance traveled by each wheel can be obtained by counting the pulses)
        // (P/U) * (3.14159*D)(For example, if the total number of pulses for motor 1 is P, the distance traveled is (P/U) * (3.14159*D))
        // (The number of pulses per revolution U for different motors can be tested manually by rotating 10 revolutions and reading the pulse count, and then taking the average value to obtain)

  static const int MOTOR_TYPE_WITHOUT_ENCODER = 0;
  static const int MOTOR_TYPE_TT = 1;
  static const int MOTOR_TYPE_N20 = 2;
  static const int MOTOR_TYPE_JGB37_520_12V_110RPM = 3; // Magnetic ring rotates 44 pulses per revolution, reduction ratio: 90, default

  };

#endif
