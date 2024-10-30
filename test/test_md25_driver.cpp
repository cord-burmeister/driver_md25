#include <gtest/gtest.h>
#include <md25_driver/md25.hpp>
#include <i2c_bus.hpp>

class MD25DriverTest : public ::testing::Test {
protected:
    void SetUp() override {
        i2c_bus = std::make_unique<I2cBus>("/dev/i2c-1");
        md25_driver = std::make_unique<md25_driver>();
    }

    std::unique_ptr<I2cBus> i2c_bus;
    std::unique_ptr<md25_driver> md25_driver;
};

TEST_F(MD25DriverTest, SetupTest) {
    bool success = md25_driver->setup(rclcpp::get_logger("MD25DriverTest"), i2c_bus);
    EXPECT_TRUE(success);
}

TEST_F(MD25DriverTest, ResetEncodersTest) {
    bool success = md25_driver->resetEncoders(rclcpp::get_logger("MD25DriverTest"), i2c_bus);
    EXPECT_TRUE(success);
}

TEST_F(MD25DriverTest, GetMotorsSpeedTest) {
    bool success = true;
    std::vector<int> speeds = md25_driver->getMotorsSpeed(rclcpp::get_logger("MD25DriverTest"), i2c_bus, success);
    EXPECT_TRUE(success);
    EXPECT_EQ(speeds.size(), 4);
}

TEST_F(MD25DriverTest, ReadEncodersTest) {
    bool success = true;
    std::vector<int> encoders = md25_driver->readEncoders(rclcpp::get_logger("MD25DriverTest"), i2c_bus, success);
    EXPECT_TRUE(success);
    EXPECT_EQ(encoders.size(), 4);
}

TEST_F(MD25DriverTest, StopMotorsTest) {
    bool success = md25_driver->stopMotors(rclcpp::get_logger("MD25DriverTest"), i2c_bus);
    EXPECT_TRUE(success);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
