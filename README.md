# bus master

> **Note:** This repository is on development mode. 

Ros driver for drivers for a mecanum robot

## Overview

The bus master project is a ROS driver for a mecanum robot. It provides functionalities to control the robot's movement, read sensor data, and publish odometry information. The project is designed to work with the MD25 motor driver and the HiWonder motor driver.

> **Note:** The MD25 driver is not integrated. There is a wiring problem i could not solve. 

### Features

- Control the robot's movement using ROS topics
- Read motor speed and encoder values
- Publish odometry information
- Support for HiWonder motor drivers

### Usage Instructions

1. Clone the repository:
   ```sh
   git clone https://github.com/yourusername/bus_master.git
   cd bus_master
   ```

2. Build the package:
   ```sh
   colcon build
   ```

3. Source the setup script:
   ```sh
   source install/setup.bash
   ```

4. Launch the bus master node:
   ```sh
   ros2 launch bus_master bus_master.launch.py
   ```

## Development Environment Setup

### Dependencies

- ROS 2 (Foxy, Galactic, or Rolling)
- colcon
- Python 3.8 or later

### Installation Steps

1. Install ROS 2:
   Follow the instructions on the [ROS 2 installation page](https://docs.ros.org/en/foxy/Installation.html) to install the desired ROS 2 distribution.

2. Install colcon:
   ```sh
   sudo apt update
   sudo apt install python3-colcon-common-extensions
   ```

3. Clone the repository and build the package:
   ```sh
   git clone https://github.com/yourusername/bus_master.git
   cd bus_master
   colcon build
   ```

4. Source the setup script:
   ```sh
   source install/setup.bash
   ```

## Examples

### Controlling the Robot

To control the robot's movement, publish geometry_msgs/Twist messages to the `cmd_vel` topic. For example, to move the robot forward at a speed of 0.5 m/s:

```sh
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Reading Motor Speed

To read the motor speed, subscribe to the `current_speed` topic. For example:

```sh
ros2 topic echo /current_speed
```

### Publishing Odometry Information

The bus master node publishes odometry information to the `odom` topic. To view the odometry data, subscribe to the `odom` topic:

```sh
ros2 topic echo /odom
```

## Troubleshooting

### Common Issues

1. **Unable to connect to the motor driver:**
   - Ensure that the I2C bus is properly connected and the device ID is correct.
   - Check the wiring and connections.

2. **Motor not responding to commands:**
   - Verify that the motor driver is powered on and functioning correctly.
   - Check the ROS topic connections and ensure that the correct messages are being published.

3. **Odometry data not being published:**
   - Ensure that the `enable_odom` parameter is set to `true` in the configuration file.
   - Check for any errors in the ROS logs.

### Solutions

- **Check ROS logs:**
  Use the following command to view the ROS logs and identify any errors:
  ```sh
  ros2 log
  ```

- **Verify connections:**
  Double-check all wiring and connections to ensure that everything is properly connected.

- **Restart the node:**
  If the node is not functioning correctly, try restarting it:
  ```sh
  ros2 lifecycle set /bus_master shutdown
  ros2 lifecycle set /bus_master configure
  ros2 lifecycle set /bus_master activate
  ```

