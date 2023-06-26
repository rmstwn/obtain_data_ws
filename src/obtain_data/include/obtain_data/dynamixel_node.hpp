#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_velocity.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_velocity.hpp"

class DynamixelNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
  
  using SetVelocity = dynamixel_sdk_custom_interfaces::msg::SetVelocity;
  using GetVelocity = dynamixel_sdk_custom_interfaces::srv::GetVelocity;

  DynamixelNode();
  virtual ~DynamixelNode();

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

  rclcpp::Subscription<SetVelocity>::SharedPtr set_velocity_subscriber_;
  rclcpp::Service<GetVelocity>::SharedPtr get_velocity_server_;

  int present_position;
  int present_velocity;
};

#endif  // READ_WRITE_NODE_HPP_s