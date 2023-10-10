#include "rclcpp/create_subscription.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class Patrol: public rclcpp::Node
{
    public:
        Patrol(): Node("robot_patrol")
        {
            laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Patrol::laser_scan_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "robot patrol node started...");
        }


    private:
        void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            laser_scan_msg = msg;
            //RCLCPP_INFO(this->get_logger(), "angle range of the laser %d", laser_scan_msg->ranges.size());
            //RCLCPP_INFO(this->get_logger(), "min ngle: %f, max angle: %f, increment: %f", msg->angle_min, msg->angle_max, msg->angle_increment);
            direction_ = this->get_angle();
            RCLCPP_INFO(this->get_logger(), "direction: %f", direction_);

        }



        //method to calculate maximum angle
        float get_angle()
        {
            float max_angle = laser_scan_msg->ranges[0];
            int max_angle_index = 0;
            float angle;

            //finding the index for the maximum distance
            for (int i= 0; i< 360; i++)
            {
                if (!std::isinf(laser_scan_msg->ranges[i]) && max_angle < (laser_scan_msg->ranges[i]) )
                {
                    max_angle = laser_scan_msg->ranges[i];
                    max_angle_index = i;
                }
            }
            
            angle =  (max_angle_index * (laser_scan_msg->angle_increment)) - (M_PI_2);

            return angle;
        }


        //member variables
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber;
        float direction_;
        sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg;
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
