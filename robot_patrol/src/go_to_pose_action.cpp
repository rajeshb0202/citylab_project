#include <climits>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>


#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_patrol/action/detail/go_to_pose__struct.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <nav_msgs/msg/odometry.hpp>




class GoToPose: public rclcpp::Node
{
public:
  using GoToPoseActionMsg = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseActionMsg>;


	//constructor
  GoToPose (const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("go_to_pose_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GoToPoseActionMsg>(
      this,
      "go_to_pose",
      std::bind(&GoToPose::handle_goal, this, _1, _2),
      std::bind(&GoToPose::handle_cancel, this, _1),
      std::bind(&GoToPose::handle_accepted, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&GoToPose::odom_callback, this, _1));
  }



private:
	//calback function for /odom subscriber
  void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.theta = getThetaFromOdometry(msg->pose.pose.orientation);
  }

	//function to convert quaternion to euler angle
  double getThetaFromOdometry(const geometry_msgs::msg::Quaternion& quaternion) {
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

	//callback method for handling goals
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToPoseActionMsg::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with position");
		goal_reached = false;
        desired_pos_ = goal->goal_pos;
        // Convert theta from degrees to radians here
        desired_pos_.theta = desired_pos_.theta * (M_PI / 180.0);
        //current_state = ALIGN_TO_GOAL;
		(void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

	//callback method for cancelling goals
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

	//callback method for handling accepted goals
  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

	//method for executing the goal
  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPoseActionMsg::Feedback>();
    auto result = std::make_shared<GoToPoseActionMsg::Result>();
    //rclcpp::Rate loop_rate(1);

    while(!goal_reached)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) 
      {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }


      // Move robot forward and send feedback
      feedback->current_pos = current_pos_;
      goal_handle->publish_feedback(feedback);


      this->goal_task();
			//loop_rate.sleep();
    
    }   //end of while loop.
 

    // Check if goal is done
    if ( goal_reached && rclcpp::ok()) 
    {
      result->status = true;
      vel_data.linear.x = 0.0;
      vel_data.angular.z = 0.0;
      publisher_->publish(vel_data);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }




    void goal_task()
    {
        double dx = desired_pos_.x - current_pos_.x;
        double dy = desired_pos_.y - current_pos_.y;
        double target_theta = atan2(dy, dx);
        double theta_diff = target_theta - current_pos_.theta;
        float distance_to_goal = std::sqrt(dx*dx + dy*dy);
        rclcpp::WallRate loop_rate(10);  

        
        if (distance_to_goal > 0.05)
        {
            //check for alignment towards the goal point
            if (std::abs(theta_diff) > 0.1) 
            {
                vel_data.angular.z = 0.6;
                vel_data.linear.x = 0;
            }
            else
            {
                vel_data.linear.x = 0.2;
                vel_data.angular.z = 0.0;
            }
        }
        else if (std::abs(desired_pos_.theta - current_pos_.theta) > 0.05)      //final allignement once it reaches the position.
        {
            vel_data.angular.z = (desired_pos_.theta - current_pos_.theta);
            vel_data.linear.x = 0.0;
        }
        else
        {
            goal_reached = true;
        }

        //publishing the velocity
        publisher_->publish(vel_data);
        loop_rate.sleep();
    }




  //member variables
  rclcpp_action::Server<GoToPoseActionMsg>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  geometry_msgs::msg::Pose2D current_pos_;          //stores the current position.
  geometry_msgs::msg::Pose2D desired_pos_;           //stores the desired position.
  geometry_msgs::msg::Twist vel_data;
  bool goal_reached= false;
  
  /*
  enum State {
  ALIGN_TO_GOAL,
  MOVE_TO_GOAL,
  ALIGN_FINAL };

  State current_state = ALIGN_TO_GOAL;
  */

};  //end of class







int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}