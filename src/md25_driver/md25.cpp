#include <md25_driver/md25.hpp>
//#include "md25.hpp"
//md25_driver::md25_driver(const rclcpp::Logger logger, const char * _i2c_file) : m_i2c_file(_i2c_file) { m_logger = logger; }
md25_driver::md25_driver( const char * _i2c_file) : m_i2c_file(_i2c_file) {  }
md25_driver::~md25_driver(){ close(m_fd);}
//---------------------------------------------
bool md25_driver::setup(const rclcpp::Logger logger){
  /*
   * For information, see this link:
   *  http://www.robot-electronics.co.uk/htm/md25i2c.htm
   * (hopefully it sticks around).
   */
  uint8_t m_buff[BUF_LEN] = {};
  lastReadEncoders = false;
  m_buff[0] = SW_VER;  /* get software version */
  if ((m_fd = open(m_i2c_file, O_RDWR)) < 0) {
    RCLCPP_ERROR(logger, "Failed to open i2c file");
    RCLCPP_ERROR(logger, m_i2c_file);
    return false;
  } else if (ioctl(m_fd, I2C_SLAVE, deviceIdFront) < 0) {
    RCLCPP_ERROR(logger, "Could not speak to I2C bus!");
    return false;
  } else if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "Could not write to i2c slave");
    return false;
  } else if (read(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "Could not read from i2c slave");
    return false;
  }
  RCLCPP_INFO(logger, "MD25 Motors front initialized with software version '%u'", m_buff[0]);
  m_software_version_front = static_cast<int>(m_buff[0]);

  if (has2Driver)
  {
    if (ioctl(m_fd, I2C_SLAVE, deviceIdRear) < 0) {
      RCLCPP_ERROR(logger, "Could not speak to I2C bus!");
      return false;
    } else if (write(m_fd, m_buff, 1) != 1) {
      RCLCPP_ERROR(logger, "Could not write to i2c slave");
      return false;
    } else if (read(m_fd, m_buff, 1) != 1) {
      RCLCPP_ERROR(logger, "Could not read from i2c slave");
      return false;
    }
    RCLCPP_INFO(logger, "MD25 Motors rear initialized with software version '%u'", m_buff[0]);
    m_software_version_rear = static_cast<int>(m_buff[0]);
  }
  return true;
}

//---------------------------------------------
int md25_driver::getSoftwareVersion(const rclcpp::Logger logger, int deviceId)
{
   return readByte(logger, deviceId, SW_VER);
}
//---------------------------------------------
int md25_driver::getBatteryVolts(const rclcpp::Logger logger, int deviceId)
{
   return readByte(logger, deviceId, VOLT);
}
//---------------------------------------------
int md25_driver::getAccelerationRate(const rclcpp::Logger logger, int deviceId)
{
   return readByte(logger, deviceId, ACC_RATE);
}
//---------------------------------------------
int md25_driver::getMode(const rclcpp::Logger logger, int deviceId)
{
   return readByte(logger, deviceId, MODE);
}
//---------------------------------------------
std::pair<int, int> md25_driver::getMotorsSpeed(const rclcpp::Logger logger, int deviceId)
{
   return readTwoBytes(logger, deviceId, SPD1);
}
//---------------------------------------------
std::pair<int, int> md25_driver::getMotorsCurrent(const rclcpp::Logger logger, int deviceId)
{
   return readTwoBytes(logger, deviceId, I1);
}
//---------------------------------------------
bool md25_driver::enableSpeedRegulation(const rclcpp::Logger logger, int deviceId)
{
  return sendCommand(logger, deviceId, ENABLE_SPEED_REG,CMD);
}
//---------------------------------------------
bool md25_driver::disableSpeedRegulation(const rclcpp::Logger logger, int deviceId)
{
  return sendCommand(logger, deviceId, DISABLE_SPEED_REG,CMD);
}
//---------------------------------------------
bool md25_driver::enableTimeout(const rclcpp::Logger logger, int deviceId)
{
  return sendCommand(logger, deviceId, ENABLE_TIMEOUT,CMD);
}
//---------------------------------------------
bool md25_driver::disableTimeout(const rclcpp::Logger logger, int deviceId)
{
  return sendCommand(logger, deviceId, DISABLE_TIMEOUT,CMD);
}
//---------------------------------------------
bool md25_driver::setMotorsSpeed(const rclcpp::Logger logger, int deviceId, int speed1,int speed2)
{
  return writeSpeed(logger, deviceId, speed1,speed2);
}
//---------------------------------------------
bool md25_driver::stopMotors(const rclcpp::Logger logger, int deviceId)
{
  return writeSpeed(logger, deviceId, STOP_SPEED,STOP_SPEED);
}
//---------------------------------------------
bool md25_driver::haltMotors(const rclcpp::Logger logger, int deviceId){
  uint8_t m_buff[BUF_LEN] = {0};
  lastReadEncoders = false;
  //ROS_INFO("HALT received, stopping motors");
  m_buff[0] = SPD1;
  m_buff[1] = STOP_SPEED;  /* this speed stops the motors */

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

//---------------------------------------------
bool md25_driver::setMode(const rclcpp::Logger logger, int deviceId, int mode)
{
  return sendCommand(logger, deviceId, mode,MODE);
}
//---------------------------------------------
bool md25_driver::setAccelerationRate(const rclcpp::Logger logger, int deviceId, int rate)
{
  return sendCommand(logger, deviceId, rate,ACC_RATE);
}
//---------------------------------------------
bool md25_driver::resetEncoders(const rclcpp::Logger logger, int deviceId){
  bool result = sendCommand(logger, deviceId, ENCODER_RESET,CMD);
  if (!result) {
    RCLCPP_ERROR(logger, "SND: Could not reset encoders");
    return false;
  }
  m_encoder_1_ticks = 0;
  m_encoder_2_ticks = 0;
  return true;
}
//-------------------------------------------------------
std::pair<int, int> md25_driver::readEncoders(const rclcpp::Logger logger, int deviceId){
  uint8_t m_buff[BUF_LEN] = {0};
  bool error = false;

    if (!selectDevice (logger, deviceId))
    {
      RCLCPP_ERROR(logger, "REs: Could select device on i2c");
      error = true;
    }
  m_buff[0] = ENC1;
  //lock.lock();
  if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "REs: Could not write to i2c");
    error = true;
  }else if (read(m_fd, m_buff, 8) != 8) {
    RCLCPP_ERROR(logger, "REs: Could not read encoder values");
    error = true;  
  }
  //lock.unlock();
  if(error){
      return std::make_pair(m_encoder_1_ticks,m_encoder_2_ticks);
  }
  int LT = (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
  int RT = (m_buff[4] << 24) + (m_buff[5] << 16) + (m_buff[6] << 8) + m_buff[7];
  int LD = LT - m_encoder_1_ticks;
  int RD = RT - m_encoder_2_ticks;
  m_encoder_2_ticks = RT;
  m_encoder_1_ticks = LT;
  if(LD > 1000 || LD < -1000){
    RCLCPP_ERROR(logger, "REs: Left encoder Jump > 1000 - %d",LD);
  }

  if(RD > 1000 || RD < -1000){
     RCLCPP_ERROR(logger, "REs: Right encoder Jump > 1000 - %d",RD);
  }
  return std::make_pair(m_encoder_1_ticks,m_encoder_2_ticks);
}
//----------------------------------------------------------
int md25_driver::readEncoder(const rclcpp::Logger logger, int deviceId, int LR){
  uint8_t m_buff[BUF_LEN] = {0};

    if (!selectDevice (logger, deviceId))
    {
      RCLCPP_ERROR(logger, "REs: Could select device on i2c");
    }

  int ticks = 0;
  if(LR == ENC1){ticks = m_encoder_1_ticks; }else{ticks = m_encoder_2_ticks;}
  m_buff[0] = LR;
  //lock.lock();
  if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "RE: Could not write to i2c");
    //lock.unlock();
    return ticks;
  } else if (read(m_fd, m_buff, 4) != 4) {
    RCLCPP_ERROR(logger, "RE: Could not read encoder values %d ",LR);
    //lock.unlock();
    return ticks;
  }
  //lock.unlock();
  ticks = (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
  return ticks;
}
//-------------------------------------------------------
bool md25_driver::writeSpeed(const rclcpp::Logger logger, int deviceId, int left,int right){
  lastReadEncoders = false;
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = SPD1;
  m_buff[1] = left;
  m_buff[2] = right;
  //lock.lock();
  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "REs: Could select device on i2c");
  }

  if (write(m_fd, m_buff, 3) != 3) {
    RCLCPP_ERROR(logger, "WS: failed to send  speed command!");
    //lock.unlock();
    return false;  
  }
  //lock.unlock();
  return true;
}
//-------------------------------------------------------
 std::pair<int, int> md25_driver::readTwoBytes(const rclcpp::Logger logger, int deviceId, int reg){
  lastReadEncoders = false;
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  lock.lock();
  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "REs: Could select device on i2c");
    lock.unlock();
  }  
  if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "R2: Could not write to i2c");
    lock.unlock();
    return std::make_pair(0 ,0);
  } else if (read(m_fd, m_buff, 2) != 2) {
    RCLCPP_ERROR(logger, "R2: Could not read register value");
    lock.unlock();
    return std::make_pair(0 ,0);
  }
  lock.unlock();
  return std::make_pair(m_buff[0] ,m_buff[1]); 
}
//-------------------------------------------------
int md25_driver::readByte(const rclcpp::Logger logger, int deviceId, int reg){
  lastReadEncoders = false;
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  lock.lock();
  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "REs: Could select device on i2c");
    lock.unlock();
  }    
  if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "R1: Could not write to i2c");
    lock.unlock();
    return 0;
  } else if (read(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "R1: Could not read register value");
    lock.unlock();
    return 0;
  }
  lock.unlock();
  return m_buff[0];
}
//-------------------------------------------------
bool md25_driver::sendCommand(const rclcpp::Logger logger, int deviceId, int value,int reg){
  lastReadEncoders = false;
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  m_buff[1] = value;
  lock.lock();
  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "failed to change device id while send command!");
    lock.unlock();
    return false;  

  }
  if (write(m_fd, m_buff, 2) != 2) {
    RCLCPP_ERROR(logger, "failed to send command!");
    lock.unlock();
    return false;  
  }
  lock.unlock();
  return true;
}

//-------------------------------------------------
// This method is addressing the device with an id. 
// it sould be callled inside a loack for save addressing.
bool md25_driver::selectDevice(const rclcpp::Logger logger, int deviceId){

  if (lastDeviceId != deviceId)
  {
    int result = ioctl(m_fd, I2C_SLAVE, deviceId);
    if (result < 0)
    {
      RCLCPP_ERROR(logger, "Switching to device !%d ", deviceId);
      return false;
    }
    lastDeviceId = deviceId;
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

