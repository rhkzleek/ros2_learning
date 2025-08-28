/***
 * @Author: cao.guanghua
 * @Date: 2025-08-27 08:43:55
 * @LastEditTime: 2025-08-27 08:43:55
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/topics/minimal_publisher/src/member_function_with_wait_for_all_acked.cpp
 */
#include <chrono>
#include <functional>
#include <cinttypes>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher_with_wait_for_all_acked"), count_(0), wait_timeout_(5000)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", rclcpp::QoS(10).reliable());

        using rclcpp::contexts::get_global_default_context;
        get_global_default_context()->add_pre_shutdown_callback(
            [this]()
            {
                this->timer_->cancel();
                this->wait_for_all_acked();
            });

        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void wait_for_all_acked()
    {
        if (publisher_->wait_for_all_acked(wait_timeout_))
        {
            RCLCPP_INFO(this->get_logger(), "ALL subscribers acknowledge messages");
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(), "Not all subscribers acknowledge messagges during %" PRId64 " ms",
                static_cast<int64_t>(wait_timeout_.count()));
        }
    }

    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    std::chrono::milliseconds wait_timeout_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto publisher = std::make_shared<MinimalPublisher>();
    rclcpp::spin(publisher);
    rclcpp::shutdown();

    return 0;
}