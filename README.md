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

### Compiling the Code
#### Step 1: Build Custom Interfaces
On the first run, comment out the line that adds the `plugins` directory in the **first** `CMakeLists.txt` file (located in `~/ros_ws/src/argos3-ros2-bridge`). This is necessary to build the custom interfaces first, as they are required for compiling the `plugins` directory.

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select argos3_ros2_bridge
```

#### Step 2: Build Plugins
Once the first build is complete, **uncomment** the line mentioned above in `CMakeLists.txt` and build again:

```bash
colcon build --packages-select argos3_ros2_bridge
```

Your `argos3_ros2_bridge` should now be successfully compiled and ready for use!
