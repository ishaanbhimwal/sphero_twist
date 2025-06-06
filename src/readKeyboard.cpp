#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <poll.h>

class TwistBoltTeleop : public rclcpp::Node
{
public:
    TwistBoltTeleop() : Node("twist_bolt_teleop")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/bolt/cmd_vel", 10);
        keyboard_loop();
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    struct termios cooked_, raw_;
    int fd_ = 0;

    void keyboard_loop()
    {
        char c;
        bool status = false;

        // Save cooked terminal settings
        tcgetattr(fd_, &cooked_);
        memcpy(&raw_, &cooked_, sizeof(struct termios));

        raw_.c_lflag &= ~(ICANON | ECHO);
        raw_.c_cc[VEOL] = 1;
        raw_.c_cc[VEOF] = 2;

        tcsetattr(fd_, TCSANOW, &raw_);

        puts("Reading from keyboard");
        puts("Use WASD keys to control the robot");
        puts("Press Ctrl+C to exit");

        struct pollfd ufd;
        ufd.fd = fd_;
        ufd.events = POLLIN;

        while (rclcpp::ok())
        {
            int num;

            if ((num = poll(&ufd, 1, 250)) < 0)
            {
                perror("poll():");
                break;
            }
            else if (num > 0)
            {
                if (read(fd_, &c, 1) < 0)
                {
                    perror("read():");
                    break;
                }

                geometry_msgs::msg::Twist twist;

                switch (c)
                {
                    case 'w':
                        puts("Forward");
                        twist.linear.x = 20.0;
                        break;
                    case 's':
                        puts("Backward");
                        twist.linear.x = -20.0;
                        break;
                    case 'a':
                        puts("Left");
                        twist.angular.z = 90.0;
                        break;
                    case 'd':
                        puts("Right");
                        twist.angular.z = -90.0;
                        break;
                    case ' ':
                        puts("Stop");
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
                        break;
                    default:
                        puts("Unknown key");
                }

                publisher_->publish(twist);
            }
        }

        // Restore terminal settings
        tcsetattr(fd_, TCSANOW, &cooked_);
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