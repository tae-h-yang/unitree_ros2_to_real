#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

class Nav2ToGo1CmdNode : public rclcpp::Node {
public:
    Nav2ToGo1CmdNode() : Node("nav2_to_go1_cmd") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Nav2ToGo1CmdNode::cmdVelCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto high_cmd_msg = std::make_shared<ros2_unitree_legged_msgs::msg::HighCmd>();

        // Set the header
        high_cmd_msg->head = {0xFE, 0xEF}; // Example header values

        // Set control level flag
        high_cmd_msg->level_flag = UNITREE_LEGGED_SDK::HIGHLEVEL; // Example value for high-level control

        // Set other fields based on cmd_vel data or default values
        high_cmd_msg->mode = 2;
        high_cmd_msg->gait_type = 2;
        high_cmd_msg->speed_level = 0; // Example value for speed level
        //high_cmd_msg->foot_raise_height = 0.0; // Example value for foot raise height
        //high_cmd_msg->body_height = 0.0; // Example value for body height
        high_cmd_msg->euler = {0.0, 0.0, 0.0}; // Example euler angles
        high_cmd_msg->velocity = {msg->linear.x, msg->linear.y}; // Example velocity values
        high_cmd_msg->yaw_speed = msg->angular.z; // Example yaw speed

        // Publish the message
        publisher_->publish(*high_cmd_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2ToGo1CmdNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
