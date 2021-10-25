# ROS2 node for the Garmin Lidar Lite V3.0
A ROS2 node to publish a range reading from the Garmin Lidar Lite V3.0
## Install
Wire the sensor as per manufacturers specifications.  Enable the i2c port on the Raspberry Pi.  A good exmple can be found at [Sparkfun Raspberry Pi SPI and I2C Tutorial](https://learn.sparkfun.com/tutorials/saprberry-pi-spi-and-i2c-tutorial/all).  Ensure that the user that will run the ROS node is in the correct group ``sudo usermod -aG i2c ubuntu``
