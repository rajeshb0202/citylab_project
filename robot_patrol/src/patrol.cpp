#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <iostream>
#include "rclcpp/timer.hpp"
#include <unistd.h>


using namespace std::chrono_literals;

class Patrol: public rclcpp::Node
{
    public:
        Patrol(): Node("robot_patrol")
        {
            laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Patrol::laser_scan_callback, this, std::placeholders::_1));
            vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
            callback_timer = this->create_wall_timer(10ms, std::bind(&Patrol::publisher_callback, this));

            RCLCPP_INFO(this->get_logger(), "robot patrol node started...");
        }


    private:
        //callback method for getting laser scan data
        void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            laser_scan_msg = msg;
            direction_ = this->get_angle();
            float degrees = direction_ * (180.0 / M_PI);
            
            //printing- angle to be rotated
            RCLCPP_INFO(this->get_logger(), "angle: %f", degrees);

            //updating the vel_data
            vel_data.linear.x = 0.1;
            vel_data.angular.z = direction_/2;
        }



        //method to calculate angle for maximum distance
        float get_angle()
        {
            float max_range = 0.0;
            int max_range_index = 0;
            float angle;

            //finding the index for the maximum distance
            for (int i= 180; i< 540; i++)
            {

                if (!std::isinf(laser_scan_msg->ranges[i]) && laser_scan_msg->ranges[359]>0.6)
                {
                    if(max_range <= (laser_scan_msg->ranges[i]))
                    {
                        max_range = laser_scan_msg->ranges[i];
                        max_range_index = i;
                        
                    }
                }
            }
            //calculating the angle
            angle = laser_scan_msg->angle_min + (max_range_index * (laser_scan_msg->angle_increment));

            return angle;
        }




        //timer callback method to publish the velocity
        void publisher_callback()
        {
            vel_publisher->publish(vel_data);
           
        }


        //member variables
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
        rclcpp::TimerBase::SharedPtr callback_timer;
        float direction_;
        sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg;
        geometry_msgs::msg::Twist vel_data;
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
