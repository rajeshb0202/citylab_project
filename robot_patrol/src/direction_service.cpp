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
            float gap_threshold= 0.4;

            //Computing right sum
            for (int i = 180; i<300; i++)
            {
                total_dist_sec_right += request->laser_data.ranges[i];

                /*adding a condition to check obstacle. if there is any obstacle, then thata section 
                would be skipped.The skipping condition is done by zeroing the the total sum of that section.*/
                if(request->laser_data.ranges[i]<gap_threshold)
                {
                    total_dist_sec_right = 0;
                    break;
                }
            }

            //Computing front sum
            for (int i = 300; i<420; i++)
            {
                total_dist_sec_front += request->laser_data.ranges[i];
                if(request->laser_data.ranges[i]<gap_threshold)
                {
                    total_dist_sec_front = 0;
                    break;
                }
            }

            //Computing left sum
            for (int i = 420; i<540; i++)
            {
                total_dist_sec_left += request->laser_data.ranges[i];
                if(request->laser_data.ranges[i]<gap_threshold)
                {
                    total_dist_sec_left = 0;
                    break;
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

        }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectionService>());
    rclcpp::shutdown();
    return 0;
}