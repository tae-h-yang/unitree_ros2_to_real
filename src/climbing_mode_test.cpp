#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    auto node = rclcpp::Node::make_shared("climbing_mode_test_node");

    rclcpp::WallRate loop_rate(500);

    long motiontime = 0;

    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;

    // rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pub =
    //     node->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);

    auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);

    float height = 0.08;
    while (rclcpp::ok())
    {

        motiontime += 2;

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

        
        if (motiontime > 2)
        {
            high_cmd_ros.mode = 2;
            high_cmd_ros.gait_type = 3;
            std::cout << "Enter command (w: increase, s: decrease, q: quit): ";
        char key;
        std::cin >> key;

        if (key == 'q')
        {
            break;
        }

        switch (key)
        {
        case 'w':
            height += 0.02;
            high_cmd_ros.foot_raise_height = height; // Move forward
            std::cout << "increased to " << high_cmd_ros.foot_raise_height << endl; 
            break;
        case 's':
            height -= 0.02;
            high_cmd_ros.foot_raise_height = height; // Move backward
            std::cout << "decreased to " << high_cmd_ros.foot_raise_height << endl;
            break;
        default:
            std::cout << "Invalid command! Please use 'w', 's' or 'q'." << std::endl;
            continue;
        }
        }

        pub->publish(high_cmd_ros);

        rclcpp::spin_some(node);

        loop_rate.sleep();
    }

    return 0;
}
