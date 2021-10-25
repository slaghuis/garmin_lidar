# ROS2 node for the Garmin Lidar Lite V3.0
A ROS2 Node to run on a Raspberry Pi.  Reads the Garmin Lidar Lite V3.0 via I2C and publishes a sensor_msgs/msg/Range topic every 500ms.

Development environment specifics:
Raspberry Pi 3
Ubuntu 20.04.2 LTS

Node can be improved by adding parameters for the i2c sensor address and sensor configuration option.

## Installation
Wire the sensor as per manufacturers specifications.  Enable the i2c port on the Raspberry Pi.  A good exmple can be found at [Sparkfun Raspberry Pi SPI and I2C Tutorial](https://learn.sparkfun.com/tutorials/saprberry-pi-spi-and-i2c-tutorial/all).  Ensure that the user that will run the ROS node is in the correct group.
```
sudo usermod -aG i2c ubuntu
```

Clone this repo into the src directory of your ROS2 workspace. See the [ros2 tutorial](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) on how to create a workspace.
```
git clone https://github.com/slaghuis/garmin_lidar.git
```
Back in the root of your ROS workspace, build and install the package.  
```
colcon build --packages-select garmin_lidar
. install/setup.bash
```
Run the package
```
ros2 run garmin_lidar lidar_node
```
See the output in a seperate terminal
```
ros2 topic echo lidar/range
```
