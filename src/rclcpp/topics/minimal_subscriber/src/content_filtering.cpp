/***
 * @Author: cao.guanghua
 * @Date: 2025-08-28 11:30:52
 * @LastEditTime: 2025-08-28 11:30:52
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/topics/minimal_subscriber/src/content_filtering.cpp
 */

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalContentFilteringSubscriber : public rclcpp::Node
{
public:
    MinimalContentFilteringSubscriber() : Node("minimal_contentfiltering_subscriber"), current_filtering_expression_("data = %0"), current_expression_parameter_("'Hello, world! 1'"), count_(10)
    {
        rclcpp::SubscriptionOptions sub_options;
        current_expression_parameter_ = "'Hello, world! " + std::to_string(count_) + "'";
        sub_options.content_filter_options.filter_expression = current_expression_parameter_;
        sub_options.content_filter_options.expression_parameters = {
            current_filtering_expression_};

        subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalContentFilteringSubscriber::topic_callback, this, _1), sub_options);

        if (!subscription_->is_cft_enabled())
        {
            RCLCPP_WARN(
                this->get_logger(), "Content filter is not enabled since it's not supported");
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Subscribed to topic \"%s\" with content filtering", subscription_->get_topic_name());
            print_expression_parameter();
        }
    }

private:
    void topic_callback(const std_msgs::msg::String &msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        if (subscription_->is_cft_enabled())
        {
            count_ = count_ + 10;
            current_expression_parameter_ = "'Hello, world! " + std::to_string(count_) + "'";
            try
            {
                subscription_->set_content_filter(current_expression_parameter_, {current_filtering_expression_});
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
        }
    }

    void print_expression_parameter(void) const
    {
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string current_filtering_expression_;
    std::string current_expression_parameter_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalContentFilteringSubscriber>());
    rclcpp::shutdown();
    return 0;
}