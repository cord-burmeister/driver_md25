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
  /// @brief Reads a byte from the specified register of the device.
  /// @param logger The logger to use for logging messages.
  /// @param deviceId The ID of the device to read from.
  /// @param reg The register to read from.
  /// @param success A reference to a boolean that will be set to true if the operation was successful, false otherwise.
  /// @return The byte read from the register.
  uint8_t readByte(const rclcpp::Logger logger, int deviceId, uint8_t reg, bool & success);

  /// @brief Reads two bytes from the specified register of the device.
  /// @param logger The logger to use for logging messages.
  /// @param deviceId The ID of the device to read from.
  /// @param reg The register to read from.
  /// @param success A reference to a boolean that will be set to true if the operation was successful, false otherwise.
  /// @return A pair of bytes read from the register.
  std::pair<uint8_t, uint8_t> readTwoBytes(const rclcpp::Logger logger, int deviceId, uint8_t reg, bool & success);

  /// @brief Reads two integers from eight bytes starting at the specified register of the device.
  /// @param logger The logger to use for logging messages.
  /// @param deviceId The ID of the device to read from.
  /// @param reg The register to start reading from.
  /// @param success A reference to a boolean that will be set to true if the operation was successful, false otherwise.
  /// @return A pair of integers read from the register.
  std::pair<int, int> readTwoIntFrom8Bytes(const rclcpp::Logger logger, int deviceId, uint8_t reg, bool & success);

  /// @brief Sends a command to the specified register of the device.
  /// @param logger The logger to use for logging messages.
  /// @param deviceId The ID of the device to send the command to.
  /// @param command The command to send.
  /// @param reg The register to send the command to.
  /// @return True if the command was successfully sent, false otherwise.
  bool sendCommand(const rclcpp::Logger logger, int deviceId, uint8_t command,uint8_t reg);

  /// @brief Selects the specified device on the I2C bus.
  /// @param logger The logger to use for logging messages.
  /// @param deviceId The ID of the device to select.
  /// @return True if the device was successfully selected, false otherwise.
  bool selectDevice(const rclcpp::Logger logger, int deviceId);

  /// @brief Reads a specified number of integers from the specified register of the device.
  /// @param logger The logger to use for logging messages.
  /// @param deviceId The ID of the device to read from.
  /// @param reg The register to read from.
  /// @param count The number of integers to read.
  /// @param success A reference to a boolean that will be set to true if the operation was successful, false otherwise.
  /// @return A vector of integers read from the register.
  std::vector<int> readIntsFromBus(const rclcpp::Logger logger, int deviceId, uint8_t reg, int count, bool & success);

  /// @brief Writes a specified number of integers to the specified register of the device.
  /// @param logger The logger to use for logging messages.
  /// @param deviceId The ID of the device to write to.
  /// @param reg The register to write to.
  /// @param values A vector of integers to write.
  /// @return True if the integers were successfully written, false otherwise.
  bool writeIntsToBus(const rclcpp::Logger logger, int deviceId, uint8_t reg, std::vector<int> values);

  /// @brief Writes a specified number of bytes to the specified register of the device.
  /// @param logger The logger to use for logging messages.
  /// @param deviceId The ID of the device to write to.
  /// @param reg The register to write to.
  /// @param values A vector of bytes to write.
  /// @return True if the bytes were successfully written, false otherwise.
  bool writeBytesToBus(const rclcpp::Logger logger, int deviceId, uint8_t reg, std::vector<uint8_t> values);

  //-------------
private:
  std::mutex lock;
  int m_fd = -1;
  int lastDeviceId = -1;
  const char * m_i2c_file      = nullptr;

  };

#endif
