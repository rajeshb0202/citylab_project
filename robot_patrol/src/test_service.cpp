#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "custom_interface/srv/get_direction.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

using CustomServiceMessage = custom_interface::srv::GetDirection;


std::shared_ptr<sensor_msgs::msg::LaserScan> scan_data = nullptr;

void scan_callback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{
  scan_data = msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //creating a node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_direction_service_client");
    
    //creating a service client
    rclcpp::Client<CustomServiceMessage>::SharedPtr client =  node->create_client<CustomServiceMessage>("/direction_service");

    //creating a subscriber for laser scan data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_ = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, scan_callback);

    
    
    //waiting for service
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    

    //sending request to server
    auto request = std::make_shared<CustomServiceMessage::Request>();

    while(!scan_data)
    {
        rclcpp::spin_some(node);
    }

    request->laser_data = *scan_data;

    auto result_future = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned: %s", result->direction.c_str());
    } else 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /direction_service");
    }
        
  
    rclcpp::shutdown();
    return 0;
}