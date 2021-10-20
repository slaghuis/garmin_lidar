// Copyright (c) 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* **********************************************************************
 * Publishes lidar/range as sensor_msgs/msg/Range.
 * ***********************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include <garmin_lidar/garmin_lidar.h>

using namespace std::chrono_literals;

class LidarPublisher : public rclcpp::Node
{
  public:
    LidarPublisher()
    : Node("lidarlite_node"), count_(0)
    {
      // Initialize i2c peripheral in the cpu core
      myLidarLite.init();

      // Optionally configure LIDAR-Lite
      myLidarLite.configure(GarminLidar::SensorMode::Default);
    
      publisher_ = this->create_publisher<sensor_msgs::msg::Range>("lidar/range", 5);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&LidarPublisher::timer_callback, this));
    }

  private:
    GarminLidar myLidarLite;
    
    void timer_callback()
    {

      if (myLidarLite.getBusyFlag() == 0x00)
      {
        // When no longer busy, immediately initialize another measurement
        // and then read the distance data from the last measurement.
        // This method will result in faster I2C rep rates.
        myLidarLite.takeRange();
        int distance = myLidarLite.readDistance();  // Range measurment in centimeters
            
        rclcpp::Time now = this->get_clock()->now();
        auto message = sensor_msgs::msg::Range();
        message.header.frame_id = "lidar";
        message.header.stamp = now;
        message.radiation_type = sensor_msgs::msg::Range::INFRARED;
        message.field_of_view = 0.001;             
        message.min_range = 00.01;                 // 10 mm.
        message.max_range = 40.00;                 // 40.0 m. Range, 70% reflective target
        
        message.range = (float) distance / 100.0;  // range in meters from centimeters

        // from https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Range.msg
        // # (Note: values < range_min or > range_max should be discarded)
        if((message.range >= message.min_range) && (message.range <= message.max_range)) {
          publisher_->publish(message);
        }

      }
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPublisher>());
  rclcpp::shutdown();
  return 0;
}
