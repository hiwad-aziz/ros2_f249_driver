cmake_minimum_required(VERSION 3.5)
project(f249driver)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_library(wiringPiLib wiringPi)

add_executable(f249driver src/f249driver.cpp)
ament_target_dependencies(f249driver rclcpp std_msgs)
target_link_libraries(f249driver ${wiringPiLib})
target_include_directories(
  f249driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include>)

install(TARGETS f249driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME}/)

ament_package()
