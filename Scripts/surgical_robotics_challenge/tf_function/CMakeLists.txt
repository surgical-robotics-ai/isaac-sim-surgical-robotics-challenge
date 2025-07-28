cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0057 NEW)

project(tf_function)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin QUIET)

if (catkin_FOUND)

  set (ROS_VERSION 1)
  catkin_python_setup()

else (catkin_FOUND)
  
  set (ROS_VERSION 2)
  find_package (ament_cmake REQUIRED)
  find_package (rclpy REQUIRED)
  ament_python_install_package (tf_function)
  ament_package ()
  
endif (catkin_FOUND)
