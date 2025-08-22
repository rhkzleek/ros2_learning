/***
 * @Author: cao.guanghua
 * @Date: 2025-08-22 17:44:26
 * @LastEditTime: 2025-08-22 17:44:26
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/topics/minimal_publisher/src/member_function_with_type_adapter.cpp
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

template <>
struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
{
    using is_specialized = std::true_type;
    using custom_type = std::string;
    using ros_message_type = std_msgs::msg::String;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.data = source;
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination = source.data;
    }
};

class MinimalPublisher : public rclcpp::Node
{
    using MyAdaptedType = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;

public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<MyAdaptedType>("topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::time_callback, this));
    }

private:
    void time_callback()
    {
        std::string message = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.c_str());
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<MyAdaptedType>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}