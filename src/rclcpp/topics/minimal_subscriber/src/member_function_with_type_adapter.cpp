/***
 * @Author: cao.guanghua
 * @Date: 2025-08-26 17:28:19
 * @LastEditTime: 2025-08-26 17:29:38
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/topics/minimal_subscriber/src/member_function_with_type_adapter.cpp
 */
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

template <>
struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
{
    using is_specialized = std::true_type;
    using custom_type = std::string;
    using ros_message_type = std_msgs::msg::String;
    /* data */

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.data = source;
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination = source.data;
    }
};

class MinimalSubscriber : public rclcpp::Node
{
    using MyAdaptedType = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;

public:
    MinimalSubscriber() : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<MyAdaptedType>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    rclcpp::Subscription<MyAdaptedType>::SharedPtr subscription_;
    void topic_callback(const std::string &msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}