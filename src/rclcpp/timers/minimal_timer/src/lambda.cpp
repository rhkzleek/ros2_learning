/***
 * @Author: cao.guanghua
 * @Date: 2025-08-28 16:13:21
 * @LastEditTime: 2025-08-28 16:13:21
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/timers/minimal_timer/src/lambda.cpp
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MinimalTimer : public rclcpp::Node
{
public:
    MinimalTimer() : Node("minimal_timer")
    {
        auto timer_callback = [this]() -> void
        { RCLCPP_INFO(this->get_logger(), "Hello, world!"); };
        timer_ = this->create_wall_timer(500ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalTimer>());
    rclcpp::shutdown();
    return 0;
}