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

/**
 * @brief The md25_driver class is responsible for controlling the MD25 motor driver.
 * It inherits from the MotorController class and provides methods to setup, reset encoders,
 * get motor speeds, read encoders, and stop motors.
 */
class md25_driver : public MotorController
{
public:
  md25_driver();
  ~md25_driver();

  /**
   * @brief Sets up the MD25 motor driver by initializing the front and rear devices,
   * setting the motor mode, and setting the acceleration rate.
   * @param logger The logger to use for logging messages.
   * @param i2c_bus The I2C bus to use for communication with the MD25 motor driver.
   * @return True if the setup was successful, false otherwise.
   */
  bool setup(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus) override;

  /**
   * @brief Resets the encoders of the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param i2c_bus The I2C bus to use for communication with the MD25 motor driver.
   * @return True if the encoders were successfully reset, false otherwise.
   */
  bool resetEncoders(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus) override;

  /**
   * @brief Gets the speeds of the motors.
   * @param logger The logger to use for logging messages.
   * @param i2c_bus The I2C bus to use for communication with the MD25 motor driver.
   * @param success A reference to a boolean that will be set to true if the operation was successful, false otherwise.
   * @return A vector containing the speeds of the motors.
   */
  std::vector<int> getMotorsSpeed(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, bool & success) override;

  /**
   * @brief Reads the encoder values of the motors.
   * @param logger The logger to use for logging messages.
   * @param i2c_bus The I2C bus to use for communication with the MD25 motor driver.
   * @param success A reference to a boolean that will be set to true if the operation was successful, false otherwise.
   * @return A vector containing the encoder values of the motors.
   */
  std::vector<int> readEncoders(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus, bool & success) override;

  /**
   * @brief Stops the motors.
   * @param logger The logger to use for logging messages.
   * @param i2c_bus The I2C bus to use for communication with the MD25 motor driver.
   * @return True if the motors were successfully stopped, false otherwise.
   */
  virtual bool stopMotors(const rclcpp::Logger logger, std::unique_ptr<I2cBus>& i2c_bus);

  /**
   * @brief Gets the software version of the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to get the software version from.
   * @return The software version of the MD25 motor driver.
   */
  int getSoftwareVersion(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Gets the battery voltage of the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to get the battery voltage from.
   * @return The battery voltage of the MD25 motor driver.
   */
  int getBatteryVolts(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Gets the acceleration rate of the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to get the acceleration rate from.
   * @return The acceleration rate of the MD25 motor driver.
   */
  int getAccelerationRate(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Gets the mode of the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to get the mode from.
   * @return The mode of the MD25 motor driver.
   */
  int getMode(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Gets the speeds of the motors.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to get the motor speeds from.
   * @return A pair containing the speeds of the motors.
   */
  std::pair<int,int> getMotorsSpeed(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Gets the currents of the motors.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to get the motor currents from.
   * @return A pair containing the currents of the motors.
   */
  std::pair<int,int> getMotorsCurrent(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Enables speed regulation for the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to enable speed regulation for.
   * @return True if speed regulation was successfully enabled, false otherwise.
   */
  bool enableSpeedRegulation(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Disables speed regulation for the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to disable speed regulation for.
   * @return True if speed regulation was successfully disabled, false otherwise.
   */
  bool disableSpeedRegulation(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Enables the timeout feature for the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to enable the timeout feature for.
   * @return True if the timeout feature was successfully enabled, false otherwise.
   */
  bool enableTimeout(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Disables the timeout feature for the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to disable the timeout feature for.
   * @return True if the timeout feature was successfully disabled, false otherwise.
   */
  bool disableTimeout(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Sets the speeds of the motors.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to set the motor speeds for.
   * @param speed1 The speed of the first motor.
   * @param speed2 The speed of the second motor.
   * @return True if the motor speeds were successfully set, false otherwise.
   */
  bool setMotorsSpeed(const rclcpp::Logger logger, int deviceId, int speed1,int speed2);

  /**
   * @brief Stops the motors.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to stop the motors for.
   * @return True if the motors were successfully stopped, false otherwise.
   */
  bool stopMotors(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Halts the motors.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to halt the motors for.
   * @return True if the motors were successfully halted, false otherwise.
   */
  bool haltMotors(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Sets the mode of the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to set the mode for.
   * @param mode The mode to set.
   * @return True if the mode was successfully set, false otherwise.
   */
  bool setMode(const rclcpp::Logger logger, int deviceId, int mode);

  /**
   * @brief Sets the acceleration rate of the MD25 motor driver.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to set the acceleration rate for.
   * @param rate The acceleration rate to set.
   * @return True if the acceleration rate was successfully set, false otherwise.
   */
  bool setAccelerationRate(const rclcpp::Logger logger, int deviceId, int rate);

  /**
   * @brief Reads the encoder values of the motors.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to read the encoder values from.
   * @return A pair containing the encoder values of the motors.
   */
  std::pair<int, int> readEncoders(const rclcpp::Logger logger, int deviceId);

  /**
   * @brief Writes the speeds of the motors.
   * @param logger The logger to use for logging messages.
   * @param deviceId The ID of the device to write the motor speeds for.
   * @param left The speed of the left motor.
   * @param right The speed of the right motor.
   * @return True if the motor speeds were successfully written, false otherwise.
   */
  bool writeSpeed(const rclcpp::Logger logger, int deviceId, int left,int right);

  /**
   * @brief Gets the ID of the front device.
   * @return The ID of the front device.
   */
  int getDeviceIdFront ();

  /**
   * @brief Gets the ID of the rear device.
   * @return The ID of the rear device.
   */
  int getDeviceIdRear ();

  /**
   * @brief Gets the ID of the front left encoder.
   * @return The ID of the front left encoder.
   */
  int getFrontLeftEncoderId ();

  /**
   * @brief Gets the ID of the front right encoder.
   * @return The ID of the front right encoder.
   */
  int getFrontRightEncoderId ();

  /**
   * @brief Gets the ID of the rear left encoder.
   * @return The ID of the rear left encoder.
   */
  int getRearLeftEncoderId ();

  /**
   * @brief Gets the ID of the rear right encoder.
   * @return The ID of the rear right encoder.
   */
  int getRearRightEncoderId ();

private:
  int motor_mode_ = 1;
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

  static const int SPD1 = 0x00;
  static const int SPD2 = 0x01;
  static const int ENC1 = 0x02;
  static const int ENC2 = 0x06;
  static const int VOLT = 0x0A;
  static const int I1 = 0x0B;
  static const int I2 = 0x0C;
  static const int SW_VER = 0x0D;
  static const int ACC_RATE = 0x0E;
  static const int MODE = 0x0F;
  static const int CMD = 0x10;
  static const int ENCODER_RESET = 0x20;
  static const int DISABLE_SPEED_REG = 0x30;
  static const int ENABLE_SPEED_REG = 0x31;
  static const int DISABLE_TIMEOUT = 0x32;
  static const int ENABLE_TIMEOUT = 0x33;
  static const int STOP_SPEED = 0x00;
};

#endif
