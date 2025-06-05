// CPP equivalent of scripts/traj_control_turtle.py

#include <chrono>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1; // Used with std::bind to create a callable object

class TurtleControl : public rclcpp::Node
{
    public:
        TurtleControl()
        :Node("simple_turtle_controller_cpp"),
        tolerance_(1.0),
        current_goal_index_(0)
        
        {
            velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, bind(&TurtleControl::update_pose, this, _1));
    
            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), bind(&TurtleControl::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "Init done");
        }
  
        void add_goal(double x, double y) {
            goals_.emplace_back(x, y);
        }

    private:
        void update_pose(const turtlesim::msg::Pose::SharedPtr msg) {
            pose_ = *msg;
        }

        double euclidean_distance(const turtlesim::msg::Pose &goal_pose) {
            double xsq = pow(goal_pose.x - pose_.x, 2);
            double ysq = pow(goal_pose.y - pose_.y, 2);
            return sqrt(xsq + ysq);
        }

        double steering_angle(const turtlesim::msg::Pose &goal_pose) {
            return atan2(goal_pose.y - pose_.y, goal_pose.x - pose_.x);
        }

        double linear_vel(const turtlesim::msg::Pose &goal_pose, double constant = 1.5) {
            return constant * euclidean_distance(goal_pose);
        }

        double angular_vel(const turtlesim::msg::Pose &goal_pose, double constant = 6.0) {
            return constant * (steering_angle(goal_pose) - pose_.theta);
        }

        void move_to_goal(double goal_x, double goal_y) {
            goal_pose_ = turtlesim::msg::Pose();
            goal_pose_->x = goal_x;
            goal_pose_->y = goal_y;

        RCLCPP_INFO(this->get_logger(), "Moving to pose %.2f %.2f", goal_x, goal_y);
        }

        void timer_callback() {
            if (!goal_pose_ && current_goal_index_ < static_cast<int>(goals_.size())) {
                auto [x, y] = goals_[current_goal_index_];
                move_to_goal(x, y);
            }

            if (!goal_pose_) {
            RCLCPP_INFO(this->get_logger(), "All goals reached");
            rclcpp::shutdown();
            return;
            }

            double distance = euclidean_distance(goal_pose_.value());
            geometry_msgs::msg::Twist vel_msg;

            if (distance >= tolerance_) {
                RCLCPP_INFO(this->get_logger(), "Calculating velocities");
                vel_msg.linear.x = linear_vel(goal_pose_.value());
                vel_msg.angular.z = angular_vel(goal_pose_.value());
                RCLCPP_INFO(this->get_logger(), "Setting linear to %.2f and angular to %.2f",
                      vel_msg.linear.x, vel_msg.angular.z);
            } else {
                // Stop and move to next goal
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                goal_pose_.reset();
                current_goal_index_++;
            }

        velocity_publisher_->publish(vel_msg);
        }

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        turtlesim::msg::Pose pose_;
        std::optional<turtlesim::msg::Pose> goal_pose_;

        double tolerance_;
        std::vector<std::pair<double, double>> goals_;
        int current_goal_index_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<TurtleControl>();

  controller->add_goal(1.0, 3.0);
  controller->add_goal(5.0, 2.0);
  controller->add_goal(2.0, 10.0);

  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
