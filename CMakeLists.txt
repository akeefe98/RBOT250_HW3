cmake_minimum_required(VERSION 3.5)
project(rbot250)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

#Cpp Executable
add_executable(homework_1 src/homework_1.cpp)
ament_target_dependencies(homework_1 rclcpp std_msgs geometry_msgs rclpy sensor_msgs trajectory_msgs tf2 tf2_ros turtlesim)


install(TARGETS
  homework_1
  DESTINATION lib/${PROJECT_NAME})

#Python module
#ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/subscriber_member_function.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
