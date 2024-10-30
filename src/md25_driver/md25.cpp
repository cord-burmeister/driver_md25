#include <i2c_bus.hpp>
#include <md25_driver/md25.hpp>

md25_driver::md25_driver( ) {}
md25_driver::~md25_driver() {}

int md25_driver::getSoftwareVersion(const rclcpp::Logger logger, int deviceId)
{
   bool success = true;
   int version = readByte(logger, deviceId, SW_VER, success);
   if (!success) {
     RCLCPP_ERROR(logger, "Failed to get software version for device %d", deviceId);
   }
   return version;
}

int md25_driver::getBatteryVolts(const rclcpp::Logger logger, int deviceId)
{
   bool success = true;
   int volts = readByte(logger, deviceId, VOLT, success);
   if (!success) {
     RCLCPP_ERROR(logger, "Failed to get battery volts for device %d", deviceId);
   }
   return volts;
}

int md25_driver::getAccelerationRate(const rclcpp::Logger logger, int deviceId)
{
   bool success = true;
   int rate = readByte(logger, deviceId, ACC_RATE, success);
   if (!success) {
     RCLCPP_ERROR(logger, "Failed to get acceleration rate for device %d", deviceId);
   }
   return rate;
}

int md25_driver::getMode(const rclcpp::Logger logger, int deviceId)
{
   bool success = true;
   int mode = readByte(logger, deviceId, MODE, success);
   if (!success) {
     RCLCPP_ERROR(logger, "Failed to get mode for device %d", deviceId);
   }
   return mode;
}

std::pair<int, int> md25_driver::getMotorsSpeed(const rclcpp::Logger logger, int deviceId)
{
   bool success = true;
   std::pair<int, int> speeds = readTwoBytes(logger, deviceId, SPD1, success);
   if (!success) {
     RCLCPP_ERROR(logger, "Failed to get motors speed for device %d", deviceId);
   }
   return speeds;
}

std::pair<int, int> md25_driver::getMotorsCurrent(const rclcpp::Logger logger, int deviceId)
{
   bool success = true;
   std::pair<int, int> currents = readTwoBytes(logger, deviceId, I1, success);
   if (!success) {
     RCLCPP_ERROR(logger, "Failed to get motors current for device %d", deviceId);
   }
   return currents;
}

bool md25_driver::enableSpeedRegulation(const rclcpp::Logger logger, int deviceId)
{
  bool success = sendCommand(logger, deviceId, ENABLE_SPEED_REG, CMD);
  if (!success) {
    RCLCPP_ERROR(logger, "Failed to enable speed regulation for device %d", deviceId);
  }
  return success;
}

bool md25_driver::disableSpeedRegulation(const rclcpp::Logger logger, int deviceId)
{
  bool success = sendCommand(logger, deviceId, DISABLE_SPEED_REG, CMD);
  if (!success) {
    RCLCPP_ERROR(logger, "Failed to disable speed regulation for device %d", deviceId);
  }
  return success;
}

bool md25_driver::enableTimeout(const rclcpp::Logger logger, int deviceId)
{
  bool success = sendCommand(logger, deviceId, ENABLE_TIMEOUT, CMD);
  if (!success) {
    RCLCPP_ERROR(logger, "Failed to enable timeout for device %d", deviceId);
  }
  return success;
}

bool md25_driver::disableTimeout(const rclcpp::Logger logger, int deviceId)
{
  bool success = sendCommand(logger, deviceId, DISABLE_TIMEOUT, CMD);
  if (!success) {
    RCLCPP_ERROR(logger, "Failed to disable timeout for device %d", deviceId);
  }
  return success;
}

bool md25_driver::setMotorsSpeed(const rclcpp::Logger logger, int deviceId, int speed1, int speed2)
{
  bool success = writeSpeed(logger, deviceId, speed1, speed2);
  if (!success) {
    RCLCPP_ERROR(logger, "Failed to set motors speed for device %d", deviceId);
  }
  return success;
}

bool md25_driver::stopMotors(const rclcpp::Logger logger, int deviceId)
{
  bool success = writeSpeed(logger, deviceId, STOP_SPEED, STOP_SPEED);
  if (!success) {
    RCLCPP_ERROR(logger, "Failed to stop motors for device %d", deviceId);
  }
  return success;
}

bool md25_driver::haltMotors(const rclcpp::Logger logger, int deviceId){
  uint8_t m_buff[BUF_LEN] = {0};
  lastReadEncoders = false;
  m_buff[0] = SPD1;
  m_buff[1] = STOP_SPEED;

  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "HALT: failed to stop robot, better go catch it!");
    return false;
  }

  if (write(m_fd, m_buff, 2) != 2) {
    RCLCPP_ERROR(logger, "HALT: failed to stop robot, better go catch it!");
    return false;
  }
  m_buff[0] = SPD2;
  m_buff[1] = STOP_SPEED;
  if (write(m_fd, m_buff, 2) != 2) {
    RCLCPP_ERROR(logger, "HALT: failed to stop robot, better go catch it!");
    return false;  
  }
  return true;
}

bool md25_driver::resetEncoders(const rclcpp::Logger logger, int deviceId){
  bool success = sendCommand(logger, deviceId, ENCODER_RESET, CMD);
  if (!success) {
    RCLCPP_ERROR(logger, "Failed to reset encoders for device %d", deviceId);
    return false;
  }
  m_encoder_1_ticks = 0;
  m_encoder_2_ticks = 0;
  return true;
}

std::pair<int, int> md25_driver::readEncoders(const rclcpp::Logger logger, int deviceId){
  uint8_t m_buff[BUF_LEN] = {0};
  bool error = false;

  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "Failed to select device %d on i2c", deviceId);
    error = true;
  }
  m_buff[0] = ENC1;
  if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "Failed to write to i2c for device %d", deviceId);
    error = true;
  } else if (read(m_fd, m_buff, 8) != 8) {
    RCLCPP_ERROR(logger, "Failed to read encoder values for device %d", deviceId);
    error = true;  
  }
  if(error){
    return std::make_pair(m_encoder_1_ticks, m_encoder_2_ticks);
  }
  int LT = (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
  int RT = (m_buff[4] << 24) + (m_buff[5] << 16) + (m_buff[6] << 8) + m_buff[7];
  int LD = LT - m_encoder_1_ticks;
  int RD = RT - m_encoder_2_ticks;
  m_encoder_2_ticks = RT;
  m_encoder_1_ticks = LT;
  if(LD > 1000 || LD < -1000){
    RCLCPP_ERROR(logger, "Left encoder jump > 1000 - %d", LD);
  }

  if(RD > 1000 || RD < -1000){
    RCLCPP_ERROR(logger, "Right encoder jump > 1000 - %d", RD);
  }
  return std::make_pair(m_encoder_1_ticks, m_encoder_2_ticks);
}

int md25_driver::readEncoder(const rclcpp::Logger logger, int deviceId, int LR){
  uint8_t m_buff[BUF_LEN] = {0};

  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "Failed to select device %d on i2c", deviceId);
  }

  int ticks = 0;
  if(LR == ENC1){ticks = m_encoder_1_ticks; }else{ticks = m_encoder_2_ticks;}
  m_buff[0] = LR;
  if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "Failed to write to i2c for device %d", deviceId);
    return ticks;
  } else if (read(m_fd, m_buff, 4) != 4) {
    RCLCPP_ERROR(logger, "Failed to read encoder values for device %d", deviceId);
    return ticks;
  }
  ticks = (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
  return ticks;
}

bool md25_driver::writeSpeed(const rclcpp::Logger logger, int deviceId, int left, int right){
  lastReadEncoders = false;
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = SPD1;
  m_buff[1] = left;
  m_buff[2] = right;

  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "Failed to select device %d on i2c", deviceId);
  }

  if (write(m_fd, m_buff, 3) != 3) {
    RCLCPP_ERROR(logger, "Failed to send speed command for device %d", deviceId);
    return false;  
  }
  return true;
}

/*
                      FRONT                        
        +---------------------------------+        
        |           +---------+           |        
        |           |         |           |        
      +---------+   | 0x58    |   +---------+      
      | | 2     |   |         |   |     1 | |      
      | |       |   +--+---+--+   |       | |      
      +---------+      +---+      +---------+      
        |                                 |        
        |                                 |        
        |                                 |        
        |                                 |        
LEFT    |                                 |  RIGHT 
        |                                 |        
        |                                 |        
        |                                 |        
        |                                 |        
        |                                 |        
      +--------+      +---+       +---------+      
      | | 1    |   +--+---+--+    |     2 | |      
      | |      |   |         |    |       | |      
      +--------+   |  0x5A   |    +---------+      
        |          |         |            |        
        |          +---------+            |        
        |                                 |        
        |                                 |        
        +---------------------------------+        
                                                   
                       REAR                        
                                                   
*/

int md25_driver::getDeviceIdFront ()
{
  return deviceIdFront;
}

int md25_driver::getDeviceIdRear ()
{
  return deviceIdRear;
}

int md25_driver::getFrontLeftEncoderId ()
{
  return 2;
}

int md25_driver::getFrontRightEncoderId ()
{
  return 1;
}

int md25_driver::getRearLeftEncoderId ()
{
  return 1;
}

int md25_driver::getRearRightEncoderId ()
{
  return 2;
}
