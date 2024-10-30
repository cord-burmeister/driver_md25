#include <gtest/gtest.h>
#include <i2c_bus.hpp>
#include <rclcpp/rclcpp.hpp>

class I2cBusTest : public ::testing::Test {
protected:
  void SetUp() override {
    i2c_bus = std::make_unique<I2cBus>("/dev/i2c-1");
  }

  std::unique_ptr<I2cBus> i2c_bus;
};

TEST_F(I2cBusTest, ReadByteSuccess) {
  bool success;
  uint8_t result = i2c_bus->readByte(rclcpp::get_logger("I2cBusTest"), 0x58, 0x00, success);
  EXPECT_TRUE(success);
  EXPECT_EQ(result, 0x00); // Replace with expected value
}

TEST_F(I2cBusTest, ReadByteFailure) {
  bool success;
  uint8_t result = i2c_bus->readByte(rclcpp::get_logger("I2cBusTest"), 0x58, 0xFF, success);
  EXPECT_FALSE(success);
  EXPECT_EQ(result, 0x00); // Replace with expected value
}

TEST_F(I2cBusTest, SendCommandSuccess) {
  bool result = i2c_bus->sendCommand(rclcpp::get_logger("I2cBusTest"), 0x58, 0x01, 0x00);
  EXPECT_TRUE(result);
}

TEST_F(I2cBusTest, SendCommandFailure) {
  bool result = i2c_bus->sendCommand(rclcpp::get_logger("I2cBusTest"), 0x58, 0x01, 0xFF);
  EXPECT_FALSE(result);
}

TEST_F(I2cBusTest, SelectDeviceSuccess) {
  bool result = i2c_bus->selectDevice(rclcpp::get_logger("I2cBusTest"), 0x58);
  EXPECT_TRUE(result);
}

TEST_F(I2cBusTest, SelectDeviceFailure) {
  bool result = i2c_bus->selectDevice(rclcpp::get_logger("I2cBusTest"), 0xFF);
  EXPECT_FALSE(result);
}

TEST_F(I2cBusTest, ReadTwoBytesSuccess) {
  bool success;
  auto result = i2c_bus->readTwoBytes(rclcpp::get_logger("I2cBusTest"), 0x58, 0x00, success);
  EXPECT_TRUE(success);
  EXPECT_EQ(result.first, 0x00); // Replace with expected value
  EXPECT_EQ(result.second, 0x00); // Replace with expected value
}

TEST_F(I2cBusTest, ReadTwoBytesFailure) {
  bool success;
  auto result = i2c_bus->readTwoBytes(rclcpp::get_logger("I2cBusTest"), 0x58, 0xFF, success);
  EXPECT_FALSE(success);
  EXPECT_EQ(result.first, 0x00); // Replace with expected value
  EXPECT_EQ(result.second, 0x00); // Replace with expected value
}

TEST_F(I2cBusTest, ReadTwoIntFrom8BytesSuccess) {
  bool success;
  auto result = i2c_bus->readTwoIntFrom8Bytes(rclcpp::get_logger("I2cBusTest"), 0x58, 0x00, success);
  EXPECT_TRUE(success);
  EXPECT_EQ(result.first, 0); // Replace with expected value
  EXPECT_EQ(result.second, 0); // Replace with expected value
}

TEST_F(I2cBusTest, ReadTwoIntFrom8BytesFailure) {
  bool success;
  auto result = i2c_bus->readTwoIntFrom8Bytes(rclcpp::get_logger("I2cBusTest"), 0x58, 0xFF, success);
  EXPECT_FALSE(success);
  EXPECT_EQ(result.first, 0); // Replace with expected value
  EXPECT_EQ(result.second, 0); // Replace with expected value
}

TEST_F(I2cBusTest, ReadIntsFromBusSuccess) {
  bool success;
  auto result = i2c_bus->readIntsFromBus(rclcpp::get_logger("I2cBusTest"), 0x58, 0x00, 2, success);
  EXPECT_TRUE(success);
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result[0], 0); // Replace with expected value
  EXPECT_EQ(result[1], 0); // Replace with expected value
}

TEST_F(I2cBusTest, ReadIntsFromBusFailure) {
  bool success;
  auto result = i2c_bus->readIntsFromBus(rclcpp::get_logger("I2cBusTest"), 0x58, 0xFF, 2, success);
  EXPECT_FALSE(success);
  EXPECT_EQ(result.size(), 0);
}

TEST_F(I2cBusTest, WriteIntsToBusSuccess) {
  std::vector<int> values = {1, 2};
  bool result = i2c_bus->writeIntsToBus(rclcpp::get_logger("I2cBusTest"), 0x58, 0x00, values);
  EXPECT_TRUE(result);
}

TEST_F(I2cBusTest, WriteIntsToBusFailure) {
  std::vector<int> values = {1, 2};
  bool result = i2c_bus->writeIntsToBus(rclcpp::get_logger("I2cBusTest"), 0x58, 0xFF, values);
  EXPECT_FALSE(result);
}

TEST_F(I2cBusTest, WriteBytesToBusSuccess) {
  std::vector<uint8_t> values = {1, 2};
  bool result = i2c_bus->writeBytesToBus(rclcpp::get_logger("I2cBusTest"), 0x58, 0x00, values);
  EXPECT_TRUE(result);
}

TEST_F(I2cBusTest, WriteBytesToBusFailure) {
  std::vector<uint8_t> values = {1, 2};
  bool result = i2c_bus->writeBytesToBus(rclcpp::get_logger("I2cBusTest"), 0x58, 0xFF, values);
  EXPECT_FALSE(result);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
