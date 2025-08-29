/***
 * @Author: cao.guanghua
 * @Date: 2025-08-28 14:16:38
 * @LastEditTime: 2025-08-28 14:16:39
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/topics/minimal_subscriber/src/time_triggered_wait_set_subscriber.cpp
 */

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TimeTriggeredWaitSetSubscriber : public rclcpp::Node
{
public:
    explicit TimeTriggeredWaitSetSubscriber(rclcpp::NodeOptions options) : Node("time_triggered_wait_set_subscriber", options)
    {
        rclcpp::CallbackGroup::SharedPtr cb_group_waitset = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

        auto subscription_options = rclcpp::SubscriptionOptions();
        subscription_options.callback_group = cb_group_waitset;
        auto subscription_callback = [this](std_msgs::msg::String::UniquePtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        };

        subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, subscription_callback, subscription_options);

        auto timer_callback = [this]() -> void
        {
            std_msgs::msg::String msg;
            rclcpp::MessageInfo msg_info;
            if (subscription_->take(msg, msg_info))
            {
                std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg);
                subscription_->handle_message(type_erased_msg, msg_info);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No message available");
            }
        };

        timer_ = this->create_wall_timer(500ms, timer_callback, cb_group_waitset);
        wait_set_.add_timer(timer_);
        thread_ = std::thread([this]() -> void
                              { spin_wait_set(); });
    }

    void spin_wait_set()
    {
        while (rclcpp::ok())
        {
            /* code */
            const auto wait_result = wait_set_.wait(501ms);
            switch (wait_result.kind())
            {
            case rclcpp::WaitResultKind::Ready:
            {
                if (wait_result.get_wait_set().get_rcl_wait_set().timers[0U])
                {
                    timer_->execute_callback();
                }
            }
            case rclcpp::WaitResultKind::Timeout:
                if (rclcpp::ok())
                {
                    RCLCPP_WARN(this->get_logger(), "Timeout. No message received after given wait-time");
                }
            default:
                RCLCPP_ERROR(this->get_logger(), "Error. Wait-set failed.");
            }
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::WaitSet wait_set_;
    std::thread thread_;
};

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(TimeTriggeredWaitSetSubscriber)