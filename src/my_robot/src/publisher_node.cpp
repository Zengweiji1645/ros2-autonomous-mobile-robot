#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode()
        : Node("publisher_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "robot_message",
            10
        );

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                std_msgs::msg::String message;

                message.data = "Hello from my robot!";

                publisher_->publish(message);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Publishing: %s",
                    message.data.c_str()
                );
            }
        );
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PublisherNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
