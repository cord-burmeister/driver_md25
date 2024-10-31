#include <i2c_bus.hpp>

I2cBus::I2cBus( const char * _i2c_file) : m_i2c_file(_i2c_file) {  }
I2cBus::~I2cBus(){ close(m_fd);}


//-------------------------------------------------------
 std::pair<uint8_t, uint8_t> I2cBus::readTwoBytes(const rclcpp::Logger logger, int deviceId, uint8_t reg, bool & success){

  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  lock.lock();
  success = true;
  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "I2cBus::readTwoBytes: Could select device on i2c");
    success = false;
    lock.unlock();
  }  
  if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "I2cBus::readTwoBytes: Could not write to i2c");
    success = false;
    lock.unlock();
    return std::make_pair(0 ,0);
  } else if (read(m_fd, m_buff, 2) != 2) {
    RCLCPP_ERROR(logger, "I2cBus::readTwoBytes: Could not read register value");
    success = false;
    lock.unlock();
    return std::make_pair(0 ,0);
  }
  lock.unlock();
  return std::make_pair(m_buff[0] ,m_buff[1]); 
}


//-------------------------------------------------------
 std::pair<int, int> I2cBus::readTwoIntFrom8Bytes(const rclcpp::Logger logger, int deviceId, uint8_t reg, bool & success){

  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  lock.lock();
  success = true;
  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "I2cBus::readTwoIntFrom8Bytes: Could select device on i2c");
    success = false;
    lock.unlock();
  }  
  if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "I2cBus::readTwoIntFrom8Bytes: Could not write to i2c");
    success = false;
    lock.unlock();
    return std::make_pair(0 ,0);
  } else if (read(m_fd, m_buff, 8) != 8) {
    RCLCPP_ERROR(logger, "I2cBus::readTwoIntFrom8Bytes: Could not read register value");
    success = false;
    lock.unlock();
    return std::make_pair(0 ,0);
  }
  lock.unlock();

  int LT = (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
  int RT = (m_buff[4] << 24) + (m_buff[5] << 16) + (m_buff[6] << 8) + m_buff[7];
  return std::make_pair(LT,RT);
}


//-------------------------------------------------------
 std::vector<int> I2cBus::readIntsFromBus(const rclcpp::Logger logger, int deviceId, uint8_t reg, int count, bool & success){

  const int maxBuffer = 24 * 4;
  int bytes = count * 4;
  std::vector<int> result;

  if (count <= 0)
  {
    RCLCPP_ERROR(logger, "I2cBus::readIntsFromBus: count parameter must be greater than zero");
    success = false;
    return result;
  }  
  if (maxBuffer < bytes)
  {
    RCLCPP_ERROR(logger, "I2cBus::readIntsFromBus: count defined buffer exceed maxBuffer");
    success = false;
    return result;
  }  

  uint8_t m_buff[maxBuffer] = {0};
  m_buff[0] = reg;
  lock.lock();
  success = true;
  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "I2cBus::readIntsFromBus: Could select device on i2c");
    success = false;
    lock.unlock();
    return result;
  }  
  if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "I2cBus::readIntsFromBus: Could not write to i2c");
    success = false;
    lock.unlock();
    return result;
  } else if (read(m_fd, m_buff, bytes) != bytes) {
    RCLCPP_ERROR(logger, "I2cBus::readIntsFromBus: Could not read register values");
    success = false;
    lock.unlock();
    return result;
  }
  lock.unlock();

  for (int i = 0; i < count; i++)
  {
    int value = (m_buff[(i*4)] << 24) + (m_buff[(i*4)+1] << 16) + (m_buff[(i*4)+2] << 8) + m_buff[(i*4)+3];
    result.push_back (value);
  }
  return result;
}


//-------------------------------------------------
uint8_t I2cBus::readByte(const rclcpp::Logger logger, int deviceId, uint8_t reg, bool & success){
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  lock.lock();
  success = true;
  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "I2cBus::readByte: Could select device on i2c");
    success = false;
    lock.unlock();
  }    
  if (write(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "I2cBus::readByte: Could not write to i2c");
    success = false;
    lock.unlock();
    return 0;
  } else if (read(m_fd, m_buff, 1) != 1) {
    RCLCPP_ERROR(logger, "I2cBus::readByte: Could not read register value");
    success = false;
    lock.unlock();
    return 0;
  }
  lock.unlock();
  return m_buff[0];
}
//-------------------------------------------------
bool I2cBus::sendCommand(const rclcpp::Logger logger, int deviceId, uint8_t value,uint8_t reg){
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  m_buff[1] = value;
  lock.lock();
  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "I2cBus::sendCommand: failed to change device id while send command!");
    lock.unlock();
    return false;  

  }
  if (write(m_fd, m_buff, 2) != 2) {
    RCLCPP_ERROR(logger, "I2cBus::sendCommand: failed to send command!");
    lock.unlock();
    return false;  
  }
  lock.unlock();
  return true;
}

bool I2cBus::writeIntsToBus(const rclcpp::Logger logger, int deviceId, uint8_t reg, std::vector<int> values)
{
  bool success = true;
  const int maxBuffer = 24 * 4 + 1;
  int bytes = values.size() * 4 + 1;
  std::vector<int> result;

  if (bytes <= 0)
  {
    RCLCPP_ERROR(logger, "I2cBus::writeIntsToBus: count parameter must be greater than zero");
    success = false;
    return success;
  }  
  if (maxBuffer < bytes)
  {
    RCLCPP_ERROR(logger, "I2cBus::writeIntsToBus: count defined buffer exceed maxBuffer");
    success = false;
    return success;
  }  

  uint8_t m_buff[maxBuffer] = {0};
  m_buff[0] = reg;
  for (int i = 0; i < (int) values.size(); i++)
  {
    // Index is the sequence of the integer i*4, because need 4 bytes per integer
    // plus the offset based on the register +1
    // plus the offset for the byte representation of the integer +0..+3
    m_buff[(i*4)+1] = (values[i] >> 24) & 0xFF;
    m_buff[(i*4)+2] = (values[i] >> 16) & 0xFF;
    m_buff[(i*4)+3] = (values[i] >> 8) & 0xFF;
    m_buff[(i*4)+4] = values[i] &  0xFF;
  }
  lock.lock();
  success = true;

  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "I2cBus::writeIntsToBus: Could select device on i2c");
    success = false;
    lock.unlock();
    return success;
  }  

  if (write(m_fd, m_buff, bytes) != bytes) {
    RCLCPP_ERROR(logger, "I2cBus::writeIntsToBus: failed to send command!");
    lock.unlock();
    return false;  
  }

  lock.unlock();
  return success;
}

bool I2cBus::writeBytesToBus(const rclcpp::Logger logger, int deviceId, uint8_t reg, std::vector<uint8_t> values)
{
  bool success = true;
  const int maxBuffer = 24 * 4 + 1;
  int bytes = values.size() + 1;

  if (bytes <= 0)
  {
    RCLCPP_ERROR(logger, "I2cBus::writeBytesToBus: count parameter must be greater than zero");
    success = false;
    return success;
  }  
  if (maxBuffer < bytes)
  {
    RCLCPP_ERROR(logger, "I2cBus::writeBytesToBus: count defined buffer exceed maxBuffer");
    success = false;
    return success;
  }  

  uint8_t m_buff[maxBuffer] = {0};
  m_buff[0] = reg;
  for (int i = 0; i < (int) values.size(); i++)
  {
    // Index is the sequence of the bytes i, because need 1 byte per uint8_t
    // plus the offset based on the register +1
    // plus the offset for the byte representation of the integer +0..+3
    m_buff[(i)+1] = values[i];
  }
  lock.lock();
  success = true;

  if (!selectDevice (logger, deviceId))
  {
    RCLCPP_ERROR(logger, "I2cBus::writeBytesToBus: Could select device on i2c");
    success = false;
    lock.unlock();
    return success;
  }  

  if (write(m_fd, m_buff, bytes) != bytes) {
    RCLCPP_ERROR(logger, "I2cBus::writeBytesToBus: failed to send command!");
    lock.unlock();
    return false;  
  }

  lock.unlock();
  return success;
}

//-------------------------------------------------
// This method is addressing the device with an id. 
// it should be called inside a lock for save addressing.
bool I2cBus::selectDevice(const rclcpp::Logger logger, int deviceId){

  if (m_fd == -1)
  {
      RCLCPP_INFO(logger, "I2cBus::selectDevice: open the bus on  %s ", m_i2c_file);
      m_fd = open (m_i2c_file, O_RDWR);
      if (m_fd < 0) {
        RCLCPP_ERROR(logger, "I2cBus::selectDevice: Failed to open the bus %s ", m_i2c_file);
      }
  }
  if (lastDeviceId != deviceId)
  {
    int result = ioctl(m_fd, I2C_SLAVE, deviceId);
    if (result < 0)
    {
      RCLCPP_ERROR(logger, "I2cBus::selectDevice: Failed to switch to device !%d ", deviceId);
      return false;
    }
    lastDeviceId = deviceId;
  }
  return true;
}



