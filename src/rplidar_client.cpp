/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)

using namespace std::chrono_literals;

class LidarClientNode : public rclcpp::Node {
  public:
    LidarClientNode(): Node("sllidar_client_node") {
      publisher_ = this->create_publisher<std_msgs::msg::Float32>("lidar/obstacle_detection", 10); // queue size of 10 to limit messages in the event of a backup
      
      // subscribes from scan topic
      // calls scanCb when it receives a message
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&LidarClientNode::scanCb, this, std::placeholders::_1)
      );

      // timer to publish every 2 seconds
      timer_ = this->create_wall_timer(
        2000ms,
        std::bind(&LidarClientNode::publishTimerCb, this)
      );
    }

  private:
    // scanCb gets called FAST, will overwhelm the publisher if we publish here
    // instead, we store the last distance and publish it at a slower rate
    void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
      int count = scan->ranges.size();
      float closest = std::numeric_limits<float>::infinity();
      // printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
      // printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

      // Threshold for obstacles (in meters)
      const float obstacle_threshold = 0.15f; // 15cm
      bool obstacle_detected = false;

      for (int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        float distance = scan->ranges[i];
        // printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, distance);
      
        // Ignore invalid measurements (0 or nan)
        if (std::isnan(distance) || distance <= 0.0f) {
          continue;
        }

        // Keep the closest distance
        if (distance < closest) {
          closest = distance;
        }

        // Check if the point is an obstacle within threshold
        if (distance < obstacle_threshold) {
          obstacle_detected = true;
          RCLCPP_WARN(this->get_logger(), "[WARNING] OBSTACLE DETECTED at angle %f°, distance %fm", degree, distance);
        }
      }

      last_distance_ = closest;
      has_data_ = true;
    }

    // Runs every 2 seconds
    void publishTimerCb() {
      if (!has_data_) {
        RCLCPP_WARN(this->get_logger(), "No LiDAR data received yet.");
        return;
      }

      // publish restults
      auto message = std_msgs::msg::Float32();
      message.data = last_distance_;
      publisher_->publish(message);

      RCLCPP_INFO(this->get_logger(), "Published closest LiDAR distance: %.3f m", last_distance_);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    float last_distance_ = 0.0f;  // store last distance
    bool has_data_ = false;
};

// -----------------------

// static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          an::SharedPtr scan) {
//   int count = scan->ranges.size();
//   printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
//   printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
//          RAD2DEG(scan->angle_max));

//   // Threshold for obstacles (in meters)
//   float obstacle_threshold = 0.15; // 15cm

//   for (int i = 0; i < count; i++) {
//     float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
//     float distance = scan->ranges[i];
//     printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, distance);
  
//     // Ignore invalid measurements (0 or nan)
//     if (std::isnan(distance) || distance <= 0.0) {
//       continue;
//     }

//     // Check if the point is an obstacle within threshold
//     if (distance < obstacle_threshold) {
//       printf("[WARNING] OBSTACLE DETECTED at angle %f°, distance %fm\n", degree, distance);
//     }
//   }
// }

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), scanCb);
  rclcpp::spin(std::make_shared<LidarClientNode>());
  rclcpp::shutdown();
  return 0;
}
