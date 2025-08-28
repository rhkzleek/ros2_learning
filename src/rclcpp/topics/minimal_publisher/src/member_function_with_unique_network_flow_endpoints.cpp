/***
 * @Author: cao.guanghua
 * @Date: 2025-08-26 19:18:26
 * @LastEditTime: 2025-08-26 19:18:27
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/topics/minimal_publisher/src/member_function_with_unique_network_flow_endpoints.cpp
 */
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher_options.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisherWithUniqueNetworkFlowEndpoints : public rclcpp::Node
{
public:
    MinimalPublisherWithUniqueNetworkFlowEndpoints() : Node("minimal_publisher_with_unique_network_flow_endpoints"), count_1_(0), count_2_(0)
    {
        auto options_1 = rclcpp::PublisherOptions();
        options_1.require_unique_network_flow_endpoints = RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED;

        publisher_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10, options_1);
        timer_1_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisherWithUniqueNetworkFlowEndpoints::timer_1_callback, this));

        publisher_2_ = this->create_publisher<std_msgs::msg::String>("topic_2", 10);
        timer_2_ = this->create_wall_timer(1000ms, std::bind(&MinimalPublisherWithUniqueNetworkFlowEndpoints::timer_2_callback, this));

        try
        {
            auto network_flow_endpoints_1 = publisher_1_->get_network_flow_endpoints();
            auto network_flow_endpoints_2 = publisher_2_->get_network_flow_endpoints();

            print_network_flow_endpoints(network_flow_endpoints_1);
            print_network_flow_endpoints(network_flow_endpoints_2);
        }
        catch (const rclcpp::exceptions::RCLError &e)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

private:
    void timer_1_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_1_);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_1_->publish(message);
    }
    void timer_2_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hej, world! " + std::to_string(count_1_);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_2_->publish(message);
    }

    void print_network_flow_endpoints(const std::vector<rclcpp::NetworkFlowEndpoint> &network_flow_endpoints) const
    {
        std::ostringstream stream;
        stream << "{\"networkFlowEndpoints\": [";
        bool comma_skip = true;
        for (auto network_flow_endpoint : network_flow_endpoints)
        {
            if (comma_skip)
            {
                comma_skip = false;
            }
            else
            {
                stream << ",";
            }
            stream << network_flow_endpoint;
        }
        stream << "]}";
        RCLCPP_INFO(this->get_logger(), "%s", stream.str().c_str());
    }

private:
    rclcpp::TimerBase::SharedPtr timer_1_;
    rclcpp::TimerBase::SharedPtr timer_2_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_;
    size_t count_1_;
    size_t count_2_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisherWithUniqueNetworkFlowEndpoints>());
    rclcpp::shutdown();
    return 0;
}