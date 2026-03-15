#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class VelocityManagerNode : public rclcpp::Node
{
public:
  VelocityManagerNode()
  : Node("velocity_manager_node"),
    has_cmd_(false),
    has_avoid_cmd_(false)
  {
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&VelocityManagerNode::cmd_callback, this, std::placeholders::_1));

    avoid_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/avoid_cmd_vel", 10,
      std::bind(&VelocityManagerNode::avoid_cmd_callback, this, std::placeholders::_1));

    final_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/final_cmd_vel", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&VelocityManagerNode::publish_final_cmd, this));

    RCLCPP_INFO(this->get_logger(), "Velocity manager node started.");
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;
    has_cmd_ = true;
  }

  void avoid_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_avoid_cmd_ = *msg;
    has_avoid_cmd_ = true;
  }

  void publish_final_cmd()
  {
    geometry_msgs::msg::Twist final_cmd;

    bool avoidance_active =
      has_avoid_cmd_ &&
      (std::abs(latest_avoid_cmd_.linear.x) > 1e-6 || std::abs(latest_avoid_cmd_.angular.z) > 1e-6);

    if (avoidance_active) {
      final_cmd = latest_avoid_cmd_;
    } else if (has_cmd_) {
      final_cmd = latest_cmd_;
    }

    final_cmd_pub_->publish(final_cmd);
  }

  bool has_cmd_;
  bool has_avoid_cmd_;

  geometry_msgs::msg::Twist latest_cmd_;
  geometry_msgs::msg::Twist latest_avoid_cmd_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr avoid_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr final_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityManagerNode>());
  rclcpp::shutdown();
  return 0;
}