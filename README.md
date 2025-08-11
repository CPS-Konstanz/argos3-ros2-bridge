# ARGoS3-ROS2-Bridge

## Downloading ARGoS3-ROS2-Bridge

To download the code, clone this repository by executing the following command:

```bash
cd ~/ros_ws/src
git clone https://github.com/CPS-Konstanz/argos3-ros2-bridge.git
```

Alternatively, you can click the **Code** button on GitHub and download the ZIP file.

Once downloaded, ensure your file structure looks like:
```
~/ros_ws/src/argos3-ros2-bridge
```

---

## Compiling ARGoS3-ROS2-Bridge

### Requirements
- **ROS2 Humble** must be installed and sourced
- **ARGoS3** must be installed

### Build

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select argos3_ros2_bridge
```

Your `argos3_ros2_bridge` should now be successfully compiled and ready for use!

### 

After building, the code will generate two shared libraries:

- **Controller**  
  `build/argos3_ros2_bridge/plugins/bridge/libargos_ros_bridge.so`

- **Loop Function**  
  `build/argos3_ros2_bridge/plugins/bridge/libloop_function.so`

When designing your experiment, set these as the **controller** and **loop function** in your ARGoS configuration.

### Notes

- **Sim Time Publishing**  
  The loop function publishes ARGoS sim time to the `/clock` topic.  
  Subscribe to `/clock` in ROS 2 to use the simulation time.

- **Trigger Mechanism**  
  The loop function publishes and subscribes to the `trigger` topic.  
  Use this topic to trigger your ROS 2 `control_step`, and respond with a  
  `name_space/trigger` topic message containing a timestamp based on the simulation time.
