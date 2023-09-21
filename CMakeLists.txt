cmake_minimum_required(VERSION 3.8)
project(mturtle_teleop_key)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mturtle_teleop_key src/mturtle_teleop_key.cpp)
target_compile_features(mturtle_teleop_key PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(mturtle_teleop_key
  geometry_msgs
  rclcpp)

install(TARGETS mturtle_teleop_key
  DESTINATION lib/${PROJECT_NAME})

ament_package()
