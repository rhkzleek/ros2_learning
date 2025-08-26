/***
 * @Author: cao.guanghua
 * @Date: 2025-08-26 17:12:02
 * @LastEditTime: 2025-08-26 17:12:03
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/topics/minimal_subscriber/src/lambda.cpp
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimaSubscriber : public rclcpp::Node
{
public:
    MinimaSubscriber() : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, [this](std_msgs::msg::String::UniquePtr msg) -> void
                                                                         { RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str()); });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimaSubscriber>());
    rclcpp::shutdown();
    return 0;
}