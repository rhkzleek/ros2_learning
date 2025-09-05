/***
 * @Author: cao.guanghua
 * @Date: 2025-09-05 15:34:05
 * @LastEditTime: 2025-09-05 15:34:09
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/action/minimal_action_client/src/not_composable_with_feedback.cpp
 */
#include <chrono>
#include <cinttypes>
#include <memory>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;
rclcpp::Node::SharedPtr g_node = nullptr;

void feedback_callback(
    rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
    RCLCPP_INFO(g_node->get_logger(), "Next number in sequence received: %" PRId32, feedback->sequence.back());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("minimal_action_client");
    auto action_client = rclcpp_action::create_client<Fibonacci>(g_node, "fibonacci");

    if (!action_client->wait_for_action_server(std::chrono::seconds(20)))
    {
        RCLCPP_ERROR(g_node->get_logger(), "Action server not available after waiting");
        return 1;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(g_node->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.feedback_callback = feedback_callback;
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(g_node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(g_node->get_logger(), "send goal call failed :(");
        return 1;
    }

    rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(g_node->get_logger(), "Goal was rejected by server");
        return 1;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    RCLCPP_INFO(g_node->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(g_node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(g_node->get_logger(), "get result call failed :(");
        return 1;
    }

    rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult wrapped_result = result_future.get();
    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(g_node->get_logger(), "Goal was aborted");
        return 1;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(g_node->get_logger(), "Goal was canceled");
        return 1;
    default:
        RCLCPP_ERROR(g_node->get_logger(), "Unknown result code");
        return 1;
    }

    RCLCPP_INFO(g_node->get_logger(), "result received");
    for (auto number : wrapped_result.result->sequence)
    {
        RCLCPP_INFO(g_node->get_logger(), "%" PRId32, number);
    }

    action_client.reset();
    g_node.reset();
    rclcpp::shutdown();
    return 0;
}