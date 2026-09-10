#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include "my_robot/Controller.h"

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
        : Node("controller_node"),
          controller(
              0.5, 0.05, 0.02,
              2.0, 0.05, 0.3
          )
    {
        target_x = 10.0;
        target_y = 10.0;

        cmd_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel",
                10
            );

        pose_subscription_ =
            this->create_subscription<geometry_msgs::msg::Pose2D>(
                "robot_pose",
                10,
                [this](const geometry_msgs::msg::Pose2D & pose)
                {
                    double v;
                    double omega;

                    double dx = target_x - pose.x;
                    double dy = target_y - pose.y;
                    double distance = std::sqrt(dx * dx + dy * dy);

                   if (distance < 0.01)
                        {
                            if (!target_reached)
                            {
                                target_reached = true;

                                controller.reset();

                                RCLCPP_INFO(
                                    this->get_logger(),
                                    "Target reached! distance=%.4f",
                                    distance
                                );
                            }

                            geometry_msgs::msg::Twist cmd;

                            cmd.linear.x = 0.0;
                            cmd.angular.z = 0.0;

                            cmd_publisher_->publish(cmd);

                            return;
                        }
                    controller.calculate(
                        target_x,
                        target_y,
                        pose.x,
                        pose.y,
                        pose.theta,
                        v,
                        omega,
                        0.1
                    );

                    geometry_msgs::msg::Twist cmd;

                    cmd.linear.x = v;
                    cmd.angular.z = omega;

                    cmd_publisher_->publish(cmd);
                   
                    RCLCPP_INFO(
                        this->get_logger(),
                        "distance=%.3f v=%.3f omega=%.3f",
                        distance,
                        v,
                        omega
                    );
                }
            );
    }

private:
    Controller controller;

    double target_x;
    double target_y;
    bool target_reached = false;
    rclcpp::Publisher<
        geometry_msgs::msg::Twist
    >::SharedPtr cmd_publisher_;

    rclcpp::Subscription<
        geometry_msgs::msg::Pose2D
    >::SharedPtr pose_subscription_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ControllerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}