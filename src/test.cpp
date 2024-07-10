#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
//#include <iostream>
//#include <chrono>
//#include <thread>

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Use 'w', 's', 'a', 'd' keys to move the robot and 'q' to quit." << std::endl;

    auto node = rclcpp::Node::make_shared("test_node");

    rclcpp::WallRate loop_rate(2);

    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;

    // rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pub =
    //     node->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);

    auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);

    while (rclcpp::ok())
    {
        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.level_flag = HIGHLEVEL;
        high_cmd_ros.mode = 0;
        high_cmd_ros.gait_type = 0;
        high_cmd_ros.speed_level = 0;
        high_cmd_ros.foot_raise_height = 0;
        high_cmd_ros.body_height = 0;
        high_cmd_ros.euler[0] = 0;
        high_cmd_ros.euler[1] = 0;
        high_cmd_ros.euler[2] = 0;
        high_cmd_ros.velocity[0] = 0.0f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.yaw_speed = 0.0f;
        high_cmd_ros.reserve = 0;
        
        pub->publish(high_cmd_ros);
        
        std::cout << "Enter command (w: forward, s: backward, a: rotate ccw, d: rotate cw, q: quit): ";
        char key;
        std::cin >> key;
        
        if (key == 'q')
        {
            break;
        }

        switch (key)
        {
        case 'w':
	          high_cmd_ros.mode = 2;
            high_cmd_ros.gait_type = 2;
            high_cmd_ros.velocity[0] = 2.0f; // Move forward
            break;
        case 's':
            high_cmd_ros.mode = 2;
            high_cmd_ros.gait_type = 2;
            high_cmd_ros.velocity[0] = -2.0f; // Move backward
            break;
        case 'a':
            high_cmd_ros.mode = 2;
            high_cmd_ros.gait_type = 2;
            high_cmd_ros.yaw_speed = 2.0f; // Rotate counterclockwise
            break;
        case 'd':
            high_cmd_ros.mode = 2;
            high_cmd_ros.gait_type = 2;
            high_cmd_ros.yaw_speed = -2.0f; // Rotate clockwise
            break;
        default:
            std::cout << "Invalid command! Please use 'w', 's', 'a', 'd' or 'q'." << std::endl;
            continue;
        }

        pub->publish(high_cmd_ros);

        // Wait for half a second
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}

