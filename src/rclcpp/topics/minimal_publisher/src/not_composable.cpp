/***
 * @Author: cao.guanghua
 * @Date: 2025-08-27 16:41:14
 * @LastEditTime: 2025-08-27 16:41:14
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/topics/minimal_publisher/src/not_composable.cpp
 */
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minimal_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
    std_msgs::msg::String message;
    auto publish_count = 0;
    rclcpp::WallRate loop_rate(500ms);

    while (rclcpp::ok())
    {
        message.data = "Hello, world! " + std::to_string(publish_count++);
        RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
        try
        {
            publisher->publish(message);
            rclcpp::spin_some(node);
        }
        catch (const rclcpp::exceptions::RCLError &e)
        {
            RCLCPP_ERROR(node->get_logger(), "unexpectedly failed with %s", e.what());
        }
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}