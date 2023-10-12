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
            this->get_angle();
            float degrees = direction_ * (180.0 / M_PI);
            
            //printing- angle to be rotated
            RCLCPP_INFO(this->get_logger(), "angle: %f", degrees);
        }



        //method to calculate angle for maximum distance
        void get_angle()
        {
            float max_avg_range = 0.0;
            int max_range_index = -1;
            int scan_range_width= 120;       
            //int search_width = 60;

            direction_ = 0;

            if (laser_scan_msg->ranges[359]<0.4)
            {

                //finding the index for the maximum distance
                for (int i= 180; i< 540; i= i + scan_range_width)
                {
                    float sum = 0.0;
                    float avg_sum = 0.0;
                    for(int j = i; j<(i+scan_range_width); j=j+5)
                    {
                        if(!std::isinf(laser_scan_msg->ranges[j]))
                        {
                            sum += laser_scan_msg->ranges[j];
                        }
                    }

                    avg_sum = sum;
                    
                    if(max_avg_range < avg_sum)
                    {
                        max_avg_range = avg_sum;
                        max_range_index = i + int(scan_range_width/2);
                    }
                    
                }
                //calculating the angle
                direction_ = laser_scan_msg->angle_min + (max_range_index * (laser_scan_msg->angle_increment));
            }
        }




        //timer callback method to publish the velocity
        void publisher_callback()
        {
            //updating the vel_data
            vel_data.linear.x = 0.1;
            vel_data.angular.z = direction_/2;
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
