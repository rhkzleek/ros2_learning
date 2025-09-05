/***
 * @Author: cao.guanghua
 * @Date: 2025-09-05 14:24:23
 * @LastEditTime: 2025-09-05 14:24:27
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/action/minimal_action_client/src/not_composable_with_cancel.cpp
 */

#include <chrono>
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minimal_action_client");
    auto action_client = rclcpp_action::create_client<Fibonacci>(node, "fibonacci");

    if (!action_client->wait_for_action_server(std::chrono::seconds(20)))
    {
        RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
        return 1;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(node->get_logger(), "Sending goal");
    auto goal_handle_future = action_client->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "send goal call failed:(");
        return 1;
    }

    rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
        return 1;
    }

    auto result_future = action_client->async_get_result(goal_handle);

    auto wait_result = rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(3));

    if (rclcpp::FutureReturnCode::TIMEOUT == wait_result)
    {
        RCLCPP_INFO(node->get_logger(), "canceling goal");
        auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
        if (rclcpp::spin_until_future_complete(node, cancel_result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "failed to cancel goal");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "goal is being canceled");
    }
    else if (rclcpp::FutureReturnCode::SUCCESS != wait_result)
    {
        RCLCPP_ERROR(node->get_logger(), "failed to get result");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
        return 1;
    }
    rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult wrapped_result = result_future.get();
    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        /* code */
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
        return 1;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
        return 1;
    default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
