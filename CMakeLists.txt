cmake_minimum_required(VERSION 3.8)
project(argos3_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(std_msgs REQUIRED)

#find_package(message_generation REQUIRED)


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

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs

)


  
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

###########
## Build ##
###########

# Added for ARGoS
add_subdirectory(plugins)

ament_package()
