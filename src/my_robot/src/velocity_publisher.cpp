#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher()
        : Node("velocity_publisher")
    {
        publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel",
                10
            );

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                geometry_msgs::msg::Twist message;

                message.linear.x = 1.0;
                message.angular.z = 0.5;

                publisher_->publish(message);

                RCLCPP_INFO(
                    this->get_logger(),
                    "v = %.2f, omega = %.2f",
                    message.linear.x,
                    message.angular.z
                );
            }
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<VelocityPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
