/***
 * @Author: cao.guanghua
 * @Date: 2025-08-28 16:24:12
 * @LastEditTime: 2025-08-28 16:24:13
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/timers/minimal_timer/src/member_function.cpp
 */
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalTimer : public rclcpp::Node
{
public:
    MinimalTimer()
        : Node("minimal_timer")
    {
        timer_ = create_wall_timer(
            500ms, std::bind(&MinimalTimer::timer_callback, this));
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello, world!");
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalTimer>());
    rclcpp::shutdown();
    return 0;
}
