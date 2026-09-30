cmake_minimum_required(VERSION 3.8)
project(armor_aim)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(armor_aim_core
  src/armor_detector.cpp
  src/ballistics.cpp
  src/serial_port.cpp
  src/target_tracker.cpp
)
target_include_directories(armor_aim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(armor_aim_core ${OpenCV_LIBRARIES})

add_executable(armor_aim_node src/armor_aim_node.cpp)
target_link_libraries(armor_aim_node armor_aim_core ${OpenCV_LIBRARIES})
ament_target_dependencies(armor_aim_node cv_bridge rclcpp sensor_msgs)

install(TARGETS armor_aim_core armor_aim_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  add_executable(armor_aim_fire_guard_test test/fire_guard_test.cpp)
  target_include_directories(armor_aim_fire_guard_test PRIVATE include)
  add_test(NAME armor_aim_fire_guard_test COMMAND armor_aim_fire_guard_test)
  add_executable(armor_aim_self_test test/self_test.cpp)
  target_link_libraries(armor_aim_self_test armor_aim_core ${OpenCV_LIBRARIES} util)
  add_test(NAME armor_aim_self_test COMMAND armor_aim_self_test)
  # 固定测试输入随源码发布；缺文件时明确失败，不能默默少跑测试。
  set(RECORDED_FRAMES "${CMAKE_CURRENT_SOURCE_DIR}/test/data/recorded_frames")
  if(NOT EXISTS "${RECORDED_FRAMES}/frame_11_RGBA.png")
    message(FATAL_ERROR "Missing recorded test fixtures: ${RECORDED_FRAMES}")
  endif()
  add_executable(armor_aim_recorded_test test/recorded_frames_test.cpp)
  target_link_libraries(armor_aim_recorded_test armor_aim_core ${OpenCV_LIBRARIES})
  add_test(NAME armor_aim_recorded_test COMMAND armor_aim_recorded_test
    "${RECORDED_FRAMES}" "${CMAKE_CURRENT_BINARY_DIR}/recorded_test_output")
endif()

ament_package()
