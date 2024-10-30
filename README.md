# driver_md25

> **Note:** This repository is on development mode. 

Ros driver for 2 MD25 drivers for a mecanum robot

## Setup

1. Clone the repository:
   ```sh
   git clone https://github.com/yourusername/driver_md25.git
   cd driver_md25
   ```

2. Install dependencies:
   ```sh
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the package:
   ```sh
   colcon build
   ```

4. Source the setup script:
   ```sh
   source install/setup.bash
   ```

## Running the Project

1. Launch the driver:
   ```sh
   ros2 launch driver_md25 bus_master.launch.py
   ```

2. To control the robot, publish to the `cmd_vel` topic:
   ```sh
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
   ```

## Examples of Usage

### Example 1: Moving Forward
To move the robot forward at a speed of 0.5 m/s:
```sh
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Example 2: Rotating
To rotate the robot at an angular speed of 0.5 rad/s:
```sh
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```
