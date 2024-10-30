cmake_minimum_required(VERSION 3.5)
project(driver_md25)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the -frtti flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti")

# Set flag depending on distro
if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined." )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(GTest REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_msgs  
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav_msgs
  map_msgs
  nav2_costmap_2d
  visualization_msgs
)

include_directories(
    include
)

install(
  DIRECTORY include/md25_driver/
  DESTINATION include/md25_driver/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


add_executable(marvin_bus
    src/i2c_bus.cpp
    src/md25_driver/md25.cpp
    src/md25_driver/main.cpp
)

target_include_directories(marvin_bus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


target_link_libraries(marvin_bus ${rclcpp_LIBRARIES})

ament_target_dependencies(marvin_bus ${DEPENDENCIES})

install(TARGETS marvin_bus
  DESTINATION lib/${PROJECT_NAME})

add_executable(test_md25_driver test/test_md25_driver.cpp)
target_link_libraries(test_md25_driver ${GTEST_LIBRARIES} pthread)
ament_add_gtest(test_md25_driver test/test_md25_driver.cpp)
ament_target_dependencies(test_md25_driver ${DEPENDENCIES})

add_executable(test_i2c_bus test/test_i2c_bus.cpp)
target_link_libraries(test_i2c_bus ${GTEST_LIBRARIES} pthread)
ament_add_gtest(test_i2c_bus test/test_i2c_bus.cpp)
ament_target_dependencies(test_i2c_bus ${DEPENDENCIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

ament_export_include_directories(include)
ament_package()
