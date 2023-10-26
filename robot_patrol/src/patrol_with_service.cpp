#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cmath>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <iostream>
#include "rclcpp/timer.hpp"
#include <unistd.h>
#include "custom_interface/srv/get_direction.hpp"


using namespace std::chrono_literals;
using CustomServiceMessage = custom_interface::srv::GetDirection;

class Patrol: public rclcpp::Node
{
    public:
        Patrol(): Node("robot_patrol")
        {
            laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Patrol::laser_scan_callback, this, std::placeholders::_1));
            vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
            client_ = this->create_client<CustomServiceMessage>("/direction_service");
            callback_timer = this->create_wall_timer(100ms, std::bind(&Patrol::publisher_callback, this));

            RCLCPP_INFO(this->get_logger(), "robot patrol node started...");
        }


    private:
        //callback method for getting laser scan data
        void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            laser_scan_msg = msg;
            received_scan_data = true;
        }



        //method to calculate direction_ for maximum distance
        void get_direction_()
        {

            //sending request to server
            auto request = std::make_shared<CustomServiceMessage::Request>();

            request->laser_data = *laser_scan_msg;

            auto result_future = client_->async_send_request(request, std::bind(&Patrol::response_callback, this,
                           std::placeholders::_1));
        }


        void response_callback(rclcpp::Client<CustomServiceMessage>::SharedFuture future)
        {
            try {
                auto result = future.get();
                direction = result->direction;
                //for debug putpose, printing the direction.
                //RCLCPP_INFO(this->get_logger(), "Service returned: %s", direction.c_str());
            } 
            catch (const std::exception &e) 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service /direction_service: %s", e.what());
            }
        }




        //timer callback method to publish the velocity
        void publisher_callback()
        {
            if (laser_scan_msg == nullptr)
            {
                return;
            }
            this->get_direction_();

            vel_data.linear.x = 0.1;
            if (direction == "front")
            {
                vel_data.angular.z = 0.0;
            }
            if (direction == "left")
            {
                vel_data.angular.z = 0.5;
            }
            if (direction == "right")
            {
                vel_data.angular.z = -0.5;
            }

            vel_publisher->publish(vel_data);
           
        }


        //member variables
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
        rclcpp::Client<CustomServiceMessage>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr callback_timer;
        std::string direction;
        sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg;
        geometry_msgs::msg::Twist vel_data;
        bool received_scan_data = false;
};




//main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;  
}
