#include <i2c_bus.hpp>
#include <md25_driver/hiwonder.hpp>

hiwonder_driver::hiwonder_driver( ) {}
hiwonder_driver::~hiwonder_driver() {}

// //---------------------------------------------
// int md25_driver::getSoftwareVersion(const rclcpp::Logger logger, int deviceId)
// {
//    return readByte(logger, deviceId, SW_VER);
// }
// //---------------------------------------------
// int md25_driver::getBatteryVolts(const rclcpp::Logger logger, int deviceId)
// {
//    return readByte(logger, deviceId, VOLT);
// }
// //---------------------------------------------
// int md25_driver::getAccelerationRate(const rclcpp::Logger logger, int deviceId)
// {
//    return readByte(logger, deviceId, ACC_RATE);
// }
// //---------------------------------------------
// int md25_driver::getMode(const rclcpp::Logger logger, int deviceId)
// {
//    return readByte(logger, deviceId, MODE);
// }
// //---------------------------------------------
// std::pair<int, int> md25_driver::getMotorsSpeed(const rclcpp::Logger logger, int deviceId)
// {
//    return readTwoBytes(logger, deviceId, SPD1);
// }
// //---------------------------------------------
// std::pair<int, int> md25_driver::getMotorsCurrent(const rclcpp::Logger logger, int deviceId)
// {
//    return readTwoBytes(logger, deviceId, I1);
// }
// //---------------------------------------------
// bool md25_driver::enableSpeedRegulation(const rclcpp::Logger logger, int deviceId)
// {
//   return sendCommand(logger, deviceId, ENABLE_SPEED_REG,CMD);
// }
// //---------------------------------------------
// bool md25_driver::disableSpeedRegulation(const rclcpp::Logger logger, int deviceId)
// {
//   return sendCommand(logger, deviceId, DISABLE_SPEED_REG,CMD);
// }
// //---------------------------------------------
// bool md25_driver::enableTimeout(const rclcpp::Logger logger, int deviceId)
// {
//   return sendCommand(logger, deviceId, ENABLE_TIMEOUT,CMD);
// }
// //---------------------------------------------
// bool md25_driver::disableTimeout(const rclcpp::Logger logger, int deviceId)
// {
//   return sendCommand(logger, deviceId, DISABLE_TIMEOUT,CMD);
// }
// //---------------------------------------------
// bool md25_driver::setMotorsSpeed(const rclcpp::Logger logger, int deviceId, int speed1,int speed2)
// {
//   return writeSpeed(logger, deviceId, speed1,speed2);
// }
// //---------------------------------------------
// bool md25_driver::stopMotors(const rclcpp::Logger logger, int deviceId)
// {
//   return writeSpeed(logger, deviceId, STOP_SPEED,STOP_SPEED);
// }
// //---------------------------------------------
// bool md25_driver::haltMotors(const rclcpp::Logger logger, int deviceId){
//   uint8_t m_buff[BUF_LEN] = {0};
//   lastReadEncoders = false;
//   //ROS_INFO("HALT received, stopping motors");
//   m_buff[0] = SPD1;
//   m_buff[1] = STOP_SPEED;  /* this speed stops the motors */

//   if (!selectDevice (logger, deviceId))
//   {
//     RCLCPP_ERROR(logger, "HALT: failed to stop robot, better go catch it!");
//     return false;
//   }

//   if (write(m_fd, m_buff, 2) != 2) {
//     RCLCPP_ERROR(logger, "HALT: failed to stop robot, better go catch it!");
//     return false;
//   }
//   m_buff[0] = SPD2;
//   m_buff[1] = STOP_SPEED;
//   if (write(m_fd, m_buff, 2) != 2) {
//     RCLCPP_ERROR(logger, "HALT: failed to stop robot, better go catch it!");
//     return false;  
//   }
//   return true;
// }

// //---------------------------------------------
// bool md25_driver::resetEncoders(const rclcpp::Logger logger, int deviceId){
//   bool result = sendCommand(logger, deviceId, ENCODER_RESET,CMD);
//   if (!result) {
//     RCLCPP_ERROR(logger, "SND: Could not reset encoders");
//     return false;
//   }
//   m_encoder_1_ticks = 0;
//   m_encoder_2_ticks = 0;
//   return true;
// }
// //-------------------------------------------------------
// std::pair<int, int> md25_driver::readEncoders(const rclcpp::Logger logger, int deviceId){
//   uint8_t m_buff[BUF_LEN] = {0};
//   bool error = false;

//     if (!selectDevice (logger, deviceId))
//     {
//       RCLCPP_ERROR(logger, "REs: Could select device on i2c");
//       error = true;
//     }
//   m_buff[0] = ENC1;
//   //lock.lock();
//   if (write(m_fd, m_buff, 1) != 1) {
//     RCLCPP_ERROR(logger, "REs: Could not write to i2c");
//     error = true;
//   }else if (read(m_fd, m_buff, 8) != 8) {
//     RCLCPP_ERROR(logger, "REs: Could not read encoder values");
//     error = true;  
//   }
//   //lock.unlock();
//   if(error){
//       return std::make_pair(m_encoder_1_ticks,m_encoder_2_ticks);
//   }
//   int LT = (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
//   int RT = (m_buff[4] << 24) + (m_buff[5] << 16) + (m_buff[6] << 8) + m_buff[7];
//   int LD = LT - m_encoder_1_ticks;
//   int RD = RT - m_encoder_2_ticks;
//   m_encoder_2_ticks = RT;
//   m_encoder_1_ticks = LT;
//   if(LD > 1000 || LD < -1000){
//     RCLCPP_ERROR(logger, "REs: Left encoder Jump > 1000 - %d",LD);
//   }

//   if(RD > 1000 || RD < -1000){
//      RCLCPP_ERROR(logger, "REs: Right encoder Jump > 1000 - %d",RD);
//   }
//   return std::make_pair(m_encoder_1_ticks,m_encoder_2_ticks);
// }
// //----------------------------------------------------------
// int md25_driver::readEncoder(const rclcpp::Logger logger, int deviceId, int LR){
//   uint8_t m_buff[BUF_LEN] = {0};

//     if (!selectDevice (logger, deviceId))
//     {
//       RCLCPP_ERROR(logger, "REs: Could select device on i2c");
//     }

//   int ticks = 0;
//   if(LR == ENC1){ticks = m_encoder_1_ticks; }else{ticks = m_encoder_2_ticks;}
//   m_buff[0] = LR;
//   //lock.lock();
//   if (write(m_fd, m_buff, 1) != 1) {
//     RCLCPP_ERROR(logger, "RE: Could not write to i2c");
//     //lock.unlock();
//     return ticks;
//   } else if (read(m_fd, m_buff, 4) != 4) {
//     RCLCPP_ERROR(logger, "RE: Could not read encoder values %d ",LR);
//     //lock.unlock();
//     return ticks;
//   }
//   //lock.unlock();
//   ticks = (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
//   return ticks;
// }
// //-------------------------------------------------------
// bool md25_driver::writeSpeed(const rclcpp::Logger logger, int deviceId, int left,int right){
//   lastReadEncoders = false;
//   uint8_t m_buff[BUF_LEN] = {0};
//   m_buff[0] = SPD1;
//   m_buff[1] = left;
//   m_buff[2] = right;
//   //lock.lock();
//   if (!selectDevice (logger, deviceId))
//   {
//     RCLCPP_ERROR(logger, "REs: Could select device on i2c");
//   }

//   if (write(m_fd, m_buff, 3) != 3) {
//     RCLCPP_ERROR(logger, "WS: failed to send  speed command!");
//     //lock.unlock();
//     return false;  
//   }
//   //lock.unlock();
//   return true;
// }

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

