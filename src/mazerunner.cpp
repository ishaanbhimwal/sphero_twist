#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TwistBoltTeleop : public rclcpp::Node
{
public:
    TwistBoltTeleop() : Node("twist_bolt_teleop")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/bolt/cmd_vel", 10);

        // Delay to give time for ROS 2 system to set up
        RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds...");
        rclcpp::sleep_for(2s);

        run_maze_script();  // Start the movement sequence
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist twist;
    void go_forward(int time)
    {
        for(int i = 0;i <= time;i++)
        {

        
        twist.linear.x = 20.0;
        twist.angular.z = 0.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        rclcpp::sleep_for(2s);
        }
        // //Stop
        twist.linear.x = 0.0;
        publisher_->publish(twist);
        //rclcpp::sleep_for(time);
    }
    void turn_left()
    {
        twist.angular.z = -90.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Turning left");
        rclcpp::sleep_for(1s);
        //stop
        twist.angular.z = 0.0;
        publisher_->publish(twist);
    }
    void turn_right()
    {
        twist.angular.z = +90.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Turning right");
        rclcpp::sleep_for(1s);
        //stop
        twist.angular.z = 0.0;
        publisher_->publish(twist);
    }
    void run_maze_script()
    {
        

        go_forward(3);
        turn_left();
        go_forward(6);
        turn_right();
        go_forward(15);
        turn_right();
        go_forward(15);
        turn_left();
        go_forward(6);
        turn_left();
        go_forward(5);
        turn_right();
        go_forward(6);

        RCLCPP_INFO(this->get_logger(), "Maze script complete");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistBoltTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


/*
OLD MOVEMENT SCRIPT 
//forward
        twist.linear.x = 20.0;
        twist.angular.z = 0.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        rclcpp::sleep_for(2s);

        // //Stop
        twist.linear.x = 0.0;
        publisher_->publish(twist);
        rclcpp::sleep_for(1s);

        //LEFT
        twist.angular.z = -90.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Turning left");
        rclcpp::sleep_for(1s);

        // // Step 4: Stop again
        twist.angular.z = 0.0;
        publisher_->publish(twist);

        // Step 1: Go forward
        twist.linear.x = 20.0;
        twist.angular.z = 0.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        rclcpp::sleep_for(8s);

        // Step 4: Stop again
        twist.angular.z = 0.0;
        publisher_->publish(twist);

        // // Step 3: Turn right
        twist.angular.z = +90.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Turning right");
        rclcpp::sleep_for(1s);
        //stop
        twist.angular.z = 0.0;
        publisher_->publish(twist);

        // //forward
        twist.linear.x = 20.0;
        twist.angular.z = 0.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        rclcpp::sleep_for(5s);

        //stop
        twist.angular.z = 0.0;
        publisher_->publish(twist);

        //right
        twist.angular.z = +90.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Turning right");
        rclcpp::sleep_for(1s);

        //stop
        twist.angular.z = 0.0;
        publisher_->publish(twist);

        //forward
        twist.linear.x = 20.0;
        twist.angular.z = 0.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        rclcpp::sleep_for(8s);

        //stop
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
        publisher_->publish(twist);

        //LEFTt
        twist.angular.z = -90.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Turning right");
        rclcpp::sleep_for(1s);

        //stop
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
        publisher_->publish(twist);

        //forward
        twist.linear.x = 20.0;
        twist.angular.z = 0.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        rclcpp::sleep_for(2s);

        //stop
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
        publisher_->publish(twist);

        //left
        twist.angular.z = -90.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Turning right");
        rclcpp::sleep_for(1s);

        //stop
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
        publisher_->publish(twist);

        //forward
        twist.linear.x = 20.0;
        twist.angular.z = 0.0;
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        rclcpp::sleep_for(2s);

        //stop
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
        publisher_->publish(twist);

*/