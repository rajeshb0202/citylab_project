#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/service.hpp"
#include "rclcpp/utilities.hpp"
#include "custom_interface/srv/get_direction.hpp"



using CustomServiceMessage = custom_interface::srv::GetDirection;

class DirectionService: public rclcpp::Node
{
    public:


        DirectionService():Node("get_direction_service_server")
        {
            srv_ = this->create_service<CustomServiceMessage>("/direction_service", std::bind(&DirectionService::compute_direction_callback, this, std::placeholders::_1, std::placeholders::_2));
        }
        
    
    
    private:
        rclcpp::Service<CustomServiceMessage>::SharedPtr srv_;

        void compute_direction_callback(const std::shared_ptr<CustomServiceMessage::Request> request, const std::shared_ptr<CustomServiceMessage::Response> response)
        {
            float total_dist_sec_left = 0, total_dist_sec_front = 0, total_dist_sec_right = 0;
            float gap_threshold= 0.6;
            bool chances_obstacles_present = false;
            int front_range = 120;


            //The robot will go straight unless there is a obstacle present on the front. 
            //If there is any obstacle present in the front, then it will change the direction.
            for (int i = 360 - front_range; i<=360+front_range; i= i+2)
            {
                if (request->laser_data.ranges[i]< gap_threshold)
                {
                    chances_obstacles_present = true;
                }
            }

            //if there is any obstacle present, then it will compute the direction.
            if (chances_obstacles_present)
            {
                //Computing right sum
                for (int i = 180; i<300; i++)
                {
                    total_dist_sec_right += request->laser_data.ranges[i];
                    if (request->laser_data.ranges[i] < gap_threshold)
                    {
                        total_dist_sec_right = 0;
                    }
                }

                //Computing front sum
                for (int i = 300; i<420; i++)
                {
                    total_dist_sec_front += request->laser_data.ranges[i];
                    if (request->laser_data.ranges[i] < gap_threshold)
                    {
                        total_dist_sec_front = 0;
                    }
                }


                //Computing left sum
                for (int i = 420; i<540; i++)
                {
                    total_dist_sec_left += request->laser_data.ranges[i];
                    if (request->laser_data.ranges[i] < gap_threshold)
                    {
                        total_dist_sec_left = 0;
                    }
                }
                

                //Deciding the response i.e. direction based upon the higher sum
                if (total_dist_sec_right >= total_dist_sec_front && total_dist_sec_right >= total_dist_sec_left)
                {
                    response->direction = "right";
                }
                else if (total_dist_sec_front >= total_dist_sec_right && total_dist_sec_front >= total_dist_sec_left)
                {
                    response->direction = "front";
                }
                else
                {
                    response->direction = "left";
                }
            } else {
            response->direction = "front";
            }

        }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectionService>());
    rclcpp::shutdown();
    return 0;
}