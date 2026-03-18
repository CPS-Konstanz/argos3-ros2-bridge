# ARGoS3-ROS2 Bridge

This package exposes ARGoS robot sensors/actuators as ROS 2 topics by loading a shared ARGoS controller plugin:

- ARGoS controller label: `argos_ros_bot_controller`
- Shared library filename: `libargos_ros_bridge.so`
- ROS 2 package name: `argos3_ros2_bridge`
- Repository folder name: `argos3-ros2-bridge` (hyphen)  
  This differs from the package name (underscore), which is normal.

This README focuses on:

1. Installing and building the bridge correctly.
2. Adding new sensors to the bridge.
3. Running ARGoS + ROS 2 controllers with correct namespace/domain mapping.
4. Operational details that are essential for stable experiments.

## 1. How The Bridge Works

For each robot in ARGoS (for example `bot0`, `bot1`, ...), the bridge:

- Creates one ROS 2 node named `argos_ros_node_<robot_id>`.
- Publishes sensor topics under `/<robot_id>/...`.
- Subscribes to actuator command topics under `/<robot_id>/...`.
- Optionally runs in lockstep with controller commands.
- Optionally publishes/subscribes bridge clock topics.

The plugin is instantiated once per robot by ARGoS through your `.argos` file.

## 2. Prerequisites

### 2.1 System Requirements

- Linux with ROS 2 (Humble or Jazzy expected).
- ARGoS3 installed and runnable (`argos3 -v` should work).
- `colcon` workspace setup.

### 2.2 Required ROS 2 And System Dependencies

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-ament-cmake \
  ros-$ROS_DISTRO-rclcpp \
  ros-$ROS_DISTRO-std-msgs \
  ros-$ROS_DISTRO-geometry-msgs \
  ros-$ROS_DISTRO-sensor-msgs \
  ros-$ROS_DISTRO-nav-msgs \
  ros-$ROS_DISTRO-tf2 \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-tf2-geometry-msgs \
  ros-$ROS_DISTRO-rosgraph-msgs \
  libgsl-dev \
  liblua5.3-dev \
  lua5.3
```

If you manage dependencies via `rosdep`:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2.3 Optional Robot Plugin Support

At compile time, the bridge checks for ARGoS plugin libraries:

- TurtleBot3 plugin: `argos3plugin_simulator_turtlebot3`
- TurtleBot4 plugin: `argos3plugin_simulator_turtlebot4`

If not found, the bridge still compiles but TurtleBot3/TurtleBot4-specific sensors are disabled.

## 3. Install And Build The Bridge

### 3.1 Clone Into A ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/CPS-Konstanz/argos3-ros2-bridge.git
```

Expected path:

```text
~/ros2_ws/src/argos3-ros2-bridge
```

### 3.2 Build

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to argos3_ros2_bridge
```

### 3.3 Source Overlay

```bash
source ~/ros2_ws/install/setup.bash
```

### 3.4 Verify Build Artifacts

Check key outputs:

```bash
ls ~/ros2_ws/install/argos3_ros2_bridge/lib/libargos_ros_bridge.so
ros2 interface show argos3_ros2_bridge/msg/LightList
```

You should also have the plugin in:

- `build/argos3_ros2_bridge/plugins/bridge/libargos_ros_bridge.so`
- `install/argos3_ros2_bridge/lib/libargos_ros_bridge.so`

## 4. ARGoS Configuration Requirements

In your `.argos` world file, define the bridge controller exactly with:

```xml
<controllers>
  <argos_ros_bot_controller id="argos_ros_bot" library="libargos_ros_bridge.so">
    <actuators>
      <differential_steering implementation="default" />
    </actuators>
    <sensors>
      <positioning implementation="default" />
    </sensors>
    <params
      multiple_domains="true"
      nodes_per_domain="50"
      ros_domain_id="0"
      enable_time_synchronization="true"
      lockstep_control="true"
      command_timeout_ms="100"
      stopWithoutSubscriberCount="10" />
  </argos_ros_bot_controller>
</controllers>
```

Then assign this controller to robots:

```xml
<entity quantity="10" max_trials="100">
  <foot-bot id="bot">
    <controller config="argos_ros_bot" />
  </foot-bot>
</entity>
```

Important:

- When `multiple_domains="true"`, robot IDs must be `bot<number>` because domain computation parses the number from that format.
- If an ID does not start with `bot`, the bridge falls back to domain 0.

## 5. Bridge Parameters (`<params ... />`)

Supported runtime parameters and defaults:

| Parameter | Type | Default | Meaning |
|---|---|---:|---|
| `multiple_domains` | bool | `false` | If `true`, compute domain per robot from robot ID. |
| `nodes_per_domain` | int | `50` | Robots per ROS domain when `multiple_domains=true`. |
| `ros_domain_id` | int | `0` | Domain used when `multiple_domains=false`. |
| `enable_time_synchronization` | bool | `true` | Enables `/<id>/argos3_clock` pub and `/<id>/ros2_clock` sub. |
| `half_baseline` | float | `0.07` | Half wheel separation used to convert `cmd_vel` to wheel speeds. |
| `wheel_radius` | float | `0.029112741` | Wheel radius for differential drive conversion. |
| `lockstep_control` | bool | `false` | Wait each ARGoS step for a fresh `cmd_vel` (until timeout). |
| `command_timeout_ms` | int | `50` | Lockstep wait timeout per step. |
| `stopWithoutSubscriberCount` | int | `10` | Stop robot if no command callback for this many ticks. |

Domain formula when `multiple_domains=true`:

```text
domain_id = robot_number / nodes_per_domain
```

Example: `nodes_per_domain=50`

- `bot0` to `bot49` -> domain 0
- `bot50` to `bot99` -> domain 1
- `bot100` to `bot149` -> domain 2

## 6. ROS 2 Topics Exposed By The Bridge

All topics are robot-scoped with prefix `/<robot_id>/`.

### 6.1 Commands (ROS 2 -> ARGoS)

| Topic | Type | Required ARGoS actuator |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `differential_steering` |
| `/cmd_led` | `argos3_ros2_bridge/msg/Led` | `leds` |
| `/cmd_rab` | `argos3_ros2_bridge/msg/Packet` | `range_and_bearing` |
| `/ros2_clock` | `rosgraph_msgs/msg/Clock` | only if `enable_time_synchronization=true` |

### 6.2 Sensor Streams (ARGoS -> ROS 2)

| Topic | Type | Required ARGoS sensor |
|---|---|---|
| `/lightList` | `argos3_ros2_bridge/msg/LightList` | `footbot_light` or `turtlebot4_light` |
| `/proximityList` | `argos3_ros2_bridge/msg/ProximityList` | `footbot_proximity`, `turtlebot3_proximity`, or `turtlebot4_proximity` |
| `/blobList` | `argos3_ros2_bridge/msg/BlobList` | `colored_blob_omnidirectional_camera` or `turtlebot4_colored_blob_omnidirectional_camera` |
| `/position` | `argos3_ros2_bridge/msg/Position` | `positioning` |
| `/odom` | `nav_msgs/msg/Odometry` | `positioning` |
| `/rab` | `argos3_ros2_bridge/msg/PacketList` | `range_and_bearing` sensor |
| `/lidarList` | `argos3_ros2_bridge/msg/LidarList` | `turtlebot3_lidar` or `turtlebot4_lidar` |
| `/scan` | `sensor_msgs/msg/LaserScan` | `turtlebot3_lidar` or `turtlebot4_lidar` |
| `/groundList` | `argos3_ros2_bridge/msg/GroundReadingList` | `turtlebot4_ground` |
| `/argos3_clock` | `rosgraph_msgs/msg/Clock` | only if `enable_time_synchronization=true` |

### 6.3 TF

For TurtleBot3 LiDAR, a static TF is published:

- parent: `/<robot_id>/base_link`
- child: `/<robot_id>/lidar`

## 7. Start The Bridge And Connect ROS 2 Controllers

This is the most important run sequence.

### 7.1 Terminal A: Start ARGoS With Bridge Plugin

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash

export BRIDGE_LIB=~/ros2_ws/install/argos3_ros2_bridge/lib
export LD_LIBRARY_PATH="/usr/local/lib/argos3:${BRIDGE_LIB}:${LD_LIBRARY_PATH}"
export ARGOS_PLUGIN_PATH="${BRIDGE_LIB}${ARGOS_PLUGIN_PATH:+:${ARGOS_PLUGIN_PATH}}"
export ROS_LOCALHOST_ONLY=0

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp argos3 -c ~/ros2_ws/src/turtlebot-flocking/launch/flocking.argos
```

If ARGoS reports it cannot load `libargos_ros_bridge.so`, your `ARGOS_PLUGIN_PATH` is wrong.

### 7.2 Terminal B: Start Controller Nodes

Controllers must match each robot in:

- namespace (`/bot0`, `/bot1`, ...)
- ROS domain (`ROS_DOMAIN_ID`)

Single robot example (`bot0`, domain 0):

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ROS_DOMAIN_ID=0 ros2 run argos3_ros2_bridge turtlebot_flocking --ros-args -r __ns:=/bot0
```

Multi-robot example for 10 robots and `nodes_per_domain=50`:

```bash
for i in $(seq 0 9); do
  domain_id=$((i / 50))
  ROS_DOMAIN_ID=$domain_id ros2 run argos3_ros2_bridge turtlebot_flocking --ros-args -r __ns:=/bot$i &
done
wait
```

Check hardcoded paths in these scripts before using them.

### 7.3 Validate Connectivity

From a shell with the correct `ROS_DOMAIN_ID` for a robot:

```bash
ros2 topic list | rg "^/bot0/"
ros2 topic info /bot0/cmd_vel
ros2 topic echo /bot0/scan --once
ros2 topic echo /bot0/odom --once
```

If `publisher count` or `subscription count` is zero where you expect nonzero, your namespace/domain mapping is not aligned.

## 8. Add New Sensors To The Bridge (Detailed Workflow)

This section describes the standard pattern used by the current bridge code.

### 8.1 Step 1: Decide Message Contract

Choose whether:

- an existing message can represent the sensor, or
- you need a new message type under `msg/`.

If you need a new message:

1. Add `msg/NewSensor.msg` (and optionally `msg/NewSensorList.msg`).
2. Register new message files in `src/argos3-ros2-bridge/CMakeLists.txt` under `set(msg_files ...)`.
3. Rebuild so ROS interfaces are generated.

### 8.2 Step 2: Add C++ Sensor Interface In `argos_ros_bridge.h`

In `plugins/bridge/argos_ros_bridge.h`:

1. Include the ARGoS sensor interface header.
2. Add member pointer for the sensor handle.
3. Add ROS publisher member for the message.

Pattern:

```cpp
// 1) ARGoS interface include
#include <argos3/plugins/robots/<robot>/control_interface/ci_<sensor>_sensor.h>

// 2) Publisher member
rclcpp::Publisher<argos3_ros2_bridge::msg::NewSensorList>::SharedPtr newSensorPublisher_;

// 3) Sensor handle
CCI_<Sensor>Sensor* m_pcNewSensor;
```

If the sensor is robot-family-specific (for example TurtleBot3/4), wrap includes and members in `#ifdef HAVE_TURTLEBOT3` or `#ifdef HAVE_TURTLEBOT4`.

### 8.3 Step 3: Initialize Sensor + Publisher In `Init(...)`

In `plugins/bridge/argos_ros_bridge.cpp`, inside `Init`:

```cpp
if (HasSensor("new_sensor_name")) {
  m_pcNewSensor = GetSensor<CCI_<Sensor>Sensor>("new_sensor_name");
  std::stringstream topic;
  topic << "/" << robot_id_ << "/newSensorList";
  newSensorPublisher_ =
    nodeHandle_->create_publisher<argos3_ros2_bridge::msg::NewSensorList>(topic.str(), 1);
}
```

Topic naming convention in this bridge is camelCase (`lightList`, `blobList`, `proximityList`, ...). Keep naming consistent.

### 8.4 Step 4: Publish Data In `ControlStep()`

Still in `argos_ros_bridge.cpp`, inside `ControlStep()`:

```cpp
if (HasSensor("new_sensor_name")) {
  const auto& reads = m_pcNewSensor->GetReadings();
  argos3_ros2_bridge::msg::NewSensorList msg;
  msg.n = static_cast<int>(reads.size());

  for (size_t i = 0; i < reads.size(); ++i) {
    argos3_ros2_bridge::msg::NewSensor sample;
    // map fields from ARGoS reading to ROS message fields
    msg.samples.push_back(sample);
  }

  newSensorPublisher_->publish(msg);
}
```

If your message has a `std_msgs/Header`, fill:

- `msg.header.stamp = nodeHandle_->now();`
- `msg.header.frame_id = robot_id_ + "/<frame>"`.

### 8.5 Step 5: Expose Sensor In `.argos` File

In your ARGoS controller definition:

```xml
<sensors>
  <new_sensor_name implementation="default" />
</sensors>
```

If this line is missing, `HasSensor("new_sensor_name")` is false and no topic is created.

### 8.6 Step 6: Rebuild And Validate

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select argos3_ros2_bridge
source install/setup.bash
```

Run ARGoS, then verify:

```bash
ros2 topic list | rg newSensor
ros2 topic echo /bot0/newSensorList --once
```

### 8.7 Step 7: Update Controller Nodes

Your ROS 2 controller must subscribe to the new topic using the same namespace and domain strategy as existing topics.

If your controller runs in namespace `/bot0`, subscribe to `/bot0/newSensorList`.

## 9. Other Details That Are Paramount In Practice

### 9.1 Namespace/Domain Mismatch Is The #1 Failure Mode

Bridge for `bot57` may run on `ROS_DOMAIN_ID=1` (with `nodes_per_domain=50`), while your controller might run on domain 0. They will not see each other.

Always align:

- controller namespace with robot ID
- controller `ROS_DOMAIN_ID` with bridge domain for that robot

### 9.2 Lockstep Mode Tradeoff

`lockstep_control=true` increases determinism but can stall each ARGoS step waiting for commands.  
Tune `command_timeout_ms` to avoid excessive waiting.

### 9.3 Safety Stop Behavior

`stopWithoutSubscriberCount` stops wheel commands after N ticks without fresh `cmd_vel` callbacks.

This is useful if controller processes crash.

### 9.4 Time Synchronization Behavior

With `enable_time_synchronization=true`, the bridge publishes `/<id>/argos3_clock` and subscribes to `/<id>/ros2_clock`.

Use these topics only if your controllers are designed for them.

### 9.5 Library Path Setup Is Mandatory

ARGoS must be able to locate:

- ARGoS core libraries (`/usr/local/lib/argos3` in many installs)
- bridge plugin library (`.../install/argos3_ros2_bridge/lib`)

If either path is missing from `LD_LIBRARY_PATH`/`ARGOS_PLUGIN_PATH`, plugin loading fails.

### 9.6 Controller Executable Packaging In This Workspace

Some controller packages in this workspace install executables into `argos3_ros2_bridge` paths.  
In those cases, the run command is:

```bash
ros2 run argos3_ros2_bridge <controller_executable>
```

Example currently available in this workspace:

```bash
ros2 pkg executables argos3_ros2_bridge
```

## 10. Optional: Synchronization Metrics Logging

The bridge provides a loop function plugin (`sync_metrics_loop_functions`) in the same shared library.

You can enable it in `.argos`:

```xml
<loop_functions library="libargos_ros_bridge.so"
                label="sync_metrics_loop_functions"
                output="sync_metrics.csv"
                enabled="true"
                write_header="true"
                flush_interval="100" />
```

This logs per-step/per-robot lockstep timing and command latency fields.

## 11. Troubleshooting Checklist

1. `argos3` fails with plugin load error:
   - Verify `ARGOS_PLUGIN_PATH` includes bridge install lib path.
   - Verify `LD_LIBRARY_PATH` includes both ARGoS and bridge lib paths.
2. No robot topics appear:
   - Confirm sensor/actuator blocks exist in `.argos`.
   - Confirm robot IDs are actually `bot...`.
3. Controller publishes but robot does not move:
   - Confirm `differential_steering` actuator exists in `.argos`.
   - Check `stopWithoutSubscriberCount`.
   - Check `cmd_vel` topic name includes robot prefix.
4. Some robots connect, others do not:
   - Domain mismatch from `multiple_domains` grouping.
5. TurtleBot LiDAR topics missing:
   - Verify ARGoS turtlebot plugin libs exist at build time.

## 12. Minimal Quickstart (Reference)

```bash
# Terminal A (bridge + ARGoS)
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
export BRIDGE_LIB=~/ros2_ws/install/argos3_ros2_bridge/lib
export LD_LIBRARY_PATH="/usr/local/lib/argos3:${BRIDGE_LIB}:${LD_LIBRARY_PATH}"
export ARGOS_PLUGIN_PATH="${BRIDGE_LIB}${ARGOS_PLUGIN_PATH:+:${ARGOS_PLUGIN_PATH}}"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp argos3 -c src/turtlebot-flocking/launch/flocking.argos
```

```bash
# Terminal B (example controller for bot0)
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=0 ros2 run argos3_ros2_bridge turtlebot_flocking --ros-args -r __ns:=/bot0
```

## Citation
If you use ARGoS3-ROS2 Bridge in your research, please cite our work:
```
@inCollection{Mkhatshwa:ANTS:2026,
   author = {Mkhatshwa, Sindiso and Zhang, Tianfu and Leopardi, Paolo and Hamann, Heiko and Reina, Andreagiovanni},
   title = {{ROS-2-ARGoS B}ridge: Scalable Simulations of Swarms of 1000 and More Robots},
   address = {Cham},
   doi={},
   pages = {in press},
   series = {LNCS},
   volume = {16515},
   editor = {{R. Gross et al.}},
   booktitle = {Swarm Intelligence (ANTS 2026)},
   publisher = {Springer},
   year = {2026}
}
```
