#ifndef TWAO_NODE_HPP_
#define TWAO_NODE_HPP_

//can============================================================
#include <iostream>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <array>
//================================================================

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

struct PacketPair {
  can_frame msg1;
  can_frame msg2;
  bool msg1_received = false;
  bool msg2_received = false;
};

namespace amr_control
{
class AmrControllorNode : public rclcpp::Node
{
  using TwistStamped = geometry_msgs::msg::TwistStamped;

public:
    AmrControllorNode();
    ~AmrControllorNode();
    bool twai_init();

    void read_and_publish();
    void create_topic();
    /*
      /cmd_vel subscriber
      /odom publisher
      /tf publisher
      생성
    */
    void clean_msg();
private:
  rclcpp::Time previous_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  double previous_pose_x_ = 0.0;
  double previous_pose_y_ = 0.0;
  double previous_theta_ = 0.0;

  int sockfd_;
  // Timeout to consider cmd_vel commands old
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
 
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;

  rclcpp::Duration cmd_vel_timeout_ = rclcpp::Duration::from_seconds(0.5);
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<TwistStamped>::SharedPtr velocity_command_subscriber_ = nullptr;
 
  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistStamped>> received_velocity_msg_ptr_{nullptr};

  rclcpp::Time previous_update_timestamp_{0};
  // publish rate limiter 40hz << 25ms
  double publish_rate_ = 40.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  //message
  nav_msgs::msg::Odometry odometry_message_;
  tf2_msgs::msg::TFMessage odometry_transform_message_;

};
};
#endif