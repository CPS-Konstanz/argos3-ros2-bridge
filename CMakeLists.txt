cmake_minimum_required(VERSION 3.8)
project(argos3_ros2_bridge)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== dependence =====
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ===== Message list =====
set(msg_files
  "msg/Led.msg"
  "msg/Blob.msg"
  "msg/BlobList.msg"
  "msg/Proximity.msg"
  "msg/ProximityList.msg"
  "msg/Light.msg"
  "msg/LightList.msg"
  "msg/Position.msg"
  "msg/Packet.msg"
  "msg/PacketList.msg"
  "msg/Signal.msg"
)

# ===== Generate interface=====
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_subdirectory(plugins)

ament_package()
