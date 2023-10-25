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
            float sum_left = 0, sum_front = 0, sum_right = 0;
            
            //Computing right sum
            for (int i = 180; i<300; i++)
            {
                sum_right += request->laser_data.ranges[i];
            }

            //Computing front sum
            for (int i = 300; i<420; i++)
            {
                sum_front += request->laser_data.ranges[i];
            }

            //Computing left sum
            for (int i = 420; i<540; i++)
            {
                sum_left += request->laser_data.ranges[i];
            }


            //Deciding the response i.e. direction based upon the higher sum
            if (sum_right >= sum_front && sum_right >= sum_left)
            {
                response->direction = "right";
            }
            else if (sum_front >= sum_right && sum_front >= sum_left)
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