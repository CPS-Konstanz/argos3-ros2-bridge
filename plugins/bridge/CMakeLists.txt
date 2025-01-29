find_package(argos3_ros2_bridge REQUIRED)

add_library(argos_ros_bridge argos_ros_bridge.h argos_ros_bridge.cpp)

target_link_libraries(argos_ros_bridge
  ${cpp_typesupport_target}
  ${ARGOS_LIBRARY_DIRS}
  ${ARGOS_LDFLAGS}
  ${rclcpp_LIBRARIES}
  /usr/local/lib/argos3/libargos3plugin_simulator_footbot.so
  /usr/local/lib/argos3/libargos3plugin_simulator_genericrobot.so
  /usr/local/lib/argos3/libargos3core_simulator.so)
  
ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs message_runtime collective_decision_making)

ament_target_dependencies(argos_ros_bridge rclcpp geometry_msgs std_msgs ARGoS argos3_ros2_bridge)

install(
    TARGETS argos_ros_bridge
    EXPORT argos_ros_bridge
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
)

