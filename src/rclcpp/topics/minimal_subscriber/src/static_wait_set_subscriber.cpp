/***
 * @Author: cao.guanghua
 * @Date: 2025-08-28 11:57:51
 * @LastEditTime: 2025-08-28 11:57:52
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/topics/minimal_subscriber/src/static_wait_set_subscriber.cpp
 */
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class StaticWaitSetSubscriber : public rclcpp::Node
{
    using MyStaticWaitSet = rclcpp::StaticWaitSet<1, 0, 0, 0, 0, 0>;

public:
    explicit StaticWaitSetSubscriber(rclcpp::NodeOptions options)
        : Node("static_wait_set_subscriber", options),
          subscription_(
              [this]()
              {
                  rclcpp::CallbackGroup::SharedPtr cb_group_waitset = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
                  auto subscription_options = rclcpp::SubscriptionOptions();
                  subscription_options.callback_group = cb_group_waitset;
                  auto subscription_callback = [this](std_msgs::msg::String::UniquePtr msg)
                  {
                      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
                  };
                  return this->create_subscription<std_msgs::msg::String>(
                      "topic",
                      10,
                      subscription_callback,
                      subscription_options);
              }()),
          wait_set_(std::array<MyStaticWaitSet::SubscriptionEntry, 1>{{{subscription_}}}),
          thread_(std::thread([this]() -> void
                              { spin_wait_set(); }))

    {
    }

    ~StaticWaitSetSubscriber()
    {
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    void spin_wait_set()
    {
        while (rclcpp::ok())
        {
            /* code */
            const auto wait_result = wait_set_.wait(std::chrono::milliseconds(501));
            switch (wait_result.kind())
            {
            case rclcpp::WaitResultKind::Ready:
            {
                std_msgs::msg::String msg;
                rclcpp::MessageInfo msg_info;
                if (subscription_->take(msg, msg_info))
                {
                    std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg);
                    subscription_->handle_message(type_erased_msg, msg_info);
                }
                break;
            }
            case rclcpp::WaitResultKind::Timeout:
                if (rclcpp::ok())
                {
                    RCLCPP_WARN(this->get_logger(), "Timeout. No message received after given wait-time");
                }
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Error. Wait-set failed.");
            }
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    MyStaticWaitSet wait_set_;
    std::thread thread_;
};

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(StaticWaitSetSubscriber)