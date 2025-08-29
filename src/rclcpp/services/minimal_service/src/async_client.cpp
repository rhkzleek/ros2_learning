/***
 * @Author: cao.guanghua
 * @Date: 2025-08-28 18:57:07
 * @LastEditTime: 2025-08-28 18:57:07
 * @LastEditors: cao.guanghua
 * @Description:
 * @FilePath: /src/rclcpp/services/minimal_service/src/async_client.cpp
 */

#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using namespace std::chrono_literals;

class ClientNode : public rclcpp::Node
{
public:
    explicit ClientNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions{}) : Node("add_two_ints_async_client", options)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        client_ = this->create_client<AddTwoInts>("add_two_ints");

        timer_ = this->create_wall_timer(5s,
                                         [this]()
                                         {
                                             std::vector<int64_t> pruned_requests;
                                             size_t n_pruned = this->client_->prune_requests_older_than(std::chrono::system_clock::now() - 5s, &pruned_requests);
                                             if (n_pruned)
                                             {
                                                 RCLCPP_INFO(
                                                     this->get_logger(),
                                                     "The server hasn't replied for more than 5s, %zu requests were discarded, "
                                                     "the discarded requests numbers are:",
                                                     n_pruned);
                                                 for (const auto &req_num : pruned_requests)
                                                 {
                                                     RCLCPP_INFO(this->get_logger(), "\t%" PRId64, req_num);
                                                 }
                                             }
                                         });
    }

    bool wait_for_servicer_server()
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        }

        return true;
    }

    void queue_async_rquest(int64_t a, int64_t b)
    {
        auto request = std::make_shared<AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        using ServiceResponseFuter = rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFutureWithRequest;

        auto response_received_callback = [logger = this->get_logger()](ServiceResponseFuter future)
        {
            auto request_response_pair = future.get();
            RCLCPP_INFO(
                logger,
                "Result of %" PRId64 " + %" PRId64 " is: %" PRId64,
                request_response_pair.first->a,
                request_response_pair.first->b,
                request_response_pair.second->sum);
        };
        auto result = client_->async_send_request(
            request, std::move(response_received_callback));
        RCLCPP_INFO(
            this->get_logger(),
            "Sending a request to the server (request_id =%" PRId64
            "), we're going to let you know the result when ready!",
            result.request_id);
    }

private:
    rclcpp::Client<AddTwoInts>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

bool read_more(std::string &buffer, const rclcpp::Logger &logger)
{
    buffer.clear();
    std::getline(std::cin, buffer);
    if (std::cin.fail() || std::cin.eof())
    {
        RCLCPP_INFO(logger, "\nProgram was interrupted, bye!");
        return false;
    }
    return true;
}

std::optional<int64_t> read_number(std::string &buffer, const rclcpp::Logger &logger)
{
    while (true)
    {
        size_t pos;
        if (!read_more(buffer, logger))
        {
            return std::nullopt;
        }

        if (buffer == "q")
        {
            return std::nullopt;
        }
        auto ret = std::stoll(buffer, &pos);
        if (ret > std::numeric_limits<int64_t>().max())
        {
            RCLCPP_INFO(
                logger,
                "The input number should be less or equal than %" PRId64
                "\n Please try again: ",
                std::numeric_limits<int64_t>().max());
            continue;
        }

        if (pos != buffer.size())
        {
            RCLCPP_INFO(
                logger,
                "The input should be a number not: %s\n Please try again: ", buffer.c_str());
            continue;
        }

        return ret;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientNode>();
    const auto &logger = node->get_logger();
    if (!node->wait_for_servicer_server())
    {
        return 1;
    }

    RCLCPP_INFO(logger, "Server is available!!!");
    RCLCPP_INFO(logger, "We are going to add two numbers, insert the first one (or 'q' to exit): ");
    int64_t a;
    int64_t b;
    std::promise<void> stop_async_spinner;
    std::thread async_spinner_thread(
        [stop_token = stop_async_spinner.get_future(), node]()
        {
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(node);
            executor.spin_until_future_complete(stop_token);
        });
    while (1)
    {
        std::string buffer;
        auto optional_number = read_number(buffer, logger);
        if (!optional_number)
        {
            break;
        }

        a = *optional_number;
        RCLCPP_INFO(logger, "Insert the second number (or 'q' to exit): ");
        optional_number = read_number(buffer, logger);
        if (!optional_number)
        {
            break;
        }
        b = *optional_number;
        node->queue_async_rquest(a, b);
        RCLCPP_INFO(
            logger,
            "You can prepare another request to add two numbers, insert the first one (or 'q' to exit):");
    }

    stop_async_spinner.set_value();
    async_spinner_thread.join();
    rclcpp::shutdown();
    return 0;
}