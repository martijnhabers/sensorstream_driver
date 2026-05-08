cmake_minimum_required(VERSION 3.16)
project(sensorstream_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------------
# ROS 2 dependencies
# -------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs)

# -------------------------
# Protobuf
# -------------------------
find_package(Protobuf REQUIRED)

# -------------------------
# libusbmuxd (direct USB connection to iOS devices, replaces iproxy)
# -------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(USBMUXD REQUIRED libusbmuxd-2.0)

# Generate C++ from sensor.proto
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)
set(PROTO_FILE ${PROTO_DIR}/sensor.proto)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILE})

# -------------------------
# Library: protocol_handler + generated protobuf
# -------------------------
add_library(protocol_handler
  src/protocol_handler.cpp
  ${PROTO_SRCS}
)

target_include_directories(protocol_handler PUBLIC
  include
  ${CMAKE_CURRENT_BINARY_DIR}   # for generated sensor.pb.h
  ${Protobuf_INCLUDE_DIRS}
  ${USBMUXD_INCLUDE_DIRS}
)

target_link_libraries(protocol_handler
  ${Protobuf_LIBRARIES}
  ${USBMUXD_LIBRARIES}
)

ament_target_dependencies(protocol_handler rclcpp)

# -------------------------
# Executable: sensorstream_node
# -------------------------
add_executable(sensorstream_node
  src/sensorstream_node.cpp
  src/banner.cpp
  src/qrcodegen.cpp
)

target_include_directories(sensorstream_node PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Protobuf_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(sensorstream_node
  protocol_handler
  ${Protobuf_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(sensorstream_node rclcpp sensor_msgs)

# -------------------------
# Install
# -------------------------
install(TARGETS sensorstream_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
