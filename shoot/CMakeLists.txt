cmake_minimum_required(VERSION 3.8)
project(shoot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(${tf2_INCLUDE_DIRS})

add_executable(shooter src/shoot.cpp)
ament_target_dependencies(shooter rclcpp std_msgs geometry_msgs)

install(TARGETS
  shooter
  DESTINATION lib/${PROJECT_NAME})

ament_package()

