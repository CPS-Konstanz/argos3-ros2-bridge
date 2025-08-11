cmake_minimum_required(VERSION 3.8)

# ===== dependence =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ARGoS REQUIRED)
find_package(GSL REQUIRED)
find_package(Lua REQUIRED)


set(LOCAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(GENERATED_MSG_INC ${PROJECT_BINARY_DIR}/rosidl_generator_cpp)

include(${ament_cmake_DIR}/../../../share/rosidl_cmake/cmake/rosidl_get_typesupport_target.cmake OPTIONAL)
if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)
endif()

find_library(ARGOS_CORE                    NAMES argos3core_simulator
             HINTS ${ARGOS_LIBRARY_DIRS} /usr/local/lib/argos3 /usr/lib/argos3)
find_library(ARGOS_PLUGIN_SIM_FOOTBOT     NAMES argos3plugin_simulator_footbot
             HINTS ${ARGOS_LIBRARY_DIRS} /usr/local/lib/argos3 /usr/lib/argos3)
find_library(ARGOS_PLUGIN_SIM_GENERICROBOT NAMES argos3plugin_simulator_genericrobot
             HINTS ${ARGOS_LIBRARY_DIRS} /usr/local/lib/argos3 /usr/lib/argos3)

# ===== Plugin 1: Controller (.so) =====
add_library(argos_ros_bridge SHARED
  ${LOCAL_INCLUDE_DIR}/argos_ros_bridge.cpp
)

target_include_directories(argos_ros_bridge
  PUBLIC
    ${LOCAL_INCLUDE_DIR}
    ${GENERATED_MSG_INC}
    ${ARGOS_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
)
if(ARGOS_LIBRARY_DIRS)
  target_link_directories(argos_ros_bridge PRIVATE ${ARGOS_LIBRARY_DIRS})
endif()

target_link_libraries(argos_ros_bridge
  ${cpp_typesupport_target}
  ${ARGOS_CORE}
  ${ARGOS_PLUGIN_SIM_FOOTBOT}
  ${ARGOS_PLUGIN_SIM_GENERICROBOT}
)
ament_target_dependencies(argos_ros_bridge
  rclcpp std_msgs geometry_msgs
)

install(TARGETS argos_ros_bridge
  LIBRARY DESTINATION lib
)

# ===== Plugin 2: LoopFunctions (.so) =====
add_library(loop_function SHARED
  ${LOCAL_INCLUDE_DIR}/bridge_loop_function.cpp
)

target_include_directories(loop_function
  PUBLIC
    ${LOCAL_INCLUDE_DIR}
    ${GENERATED_MSG_INC}
    ${ARGOS_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
)
if(ARGOS_LIBRARY_DIRS)
  target_link_directories(loop_function PRIVATE ${ARGOS_LIBRARY_DIRS})
endif()
target_link_libraries(loop_function
  ${cpp_typesupport_target}
  ${ARGOS_CORE}
  ${ARGOS_PLUGIN_SIM_FOOTBOT}
  ${ARGOS_PLUGIN_SIM_GENERICROBOT}
)
ament_target_dependencies(loop_function
  rclcpp std_msgs
)

install(TARGETS loop_function
  LIBRARY DESTINATION lib
)
