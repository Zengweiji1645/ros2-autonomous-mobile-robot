#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include "my_robot/Robot.h"

class RobotNode : public rclcpp::Node
{
public:
    RobotNode()
        : Node("robot_node"),
          robot(0.0, 0.0, 0.0),
          v(0.0),
          omega(0.0)
    {
        // 订阅 Controller 发来的速度
        subscription_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel",
                10,
                [this](const geometry_msgs::msg::Twist & msg)
                {
                    v = msg.linear.x;
                    omega = msg.angular.z;
                }
            );

        // 发布机器人当前位置
        pose_publisher_ =
            this->create_publisher<geometry_msgs::msg::Pose2D>(
                "robot_pose",
                10
            );

        // 每 0.1 秒更新一次机器人
        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]()
                {
                    double dt = 0.1;

                    robot.update(v, omega, dt);

                    geometry_msgs::msg::Pose2D pose;

                    pose.x = robot.x;
                    pose.y = robot.y;
                    pose.theta = robot.theta;

                    pose_publisher_->publish(pose);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "x=%.2f y=%.2f theta=%.2f",
                        robot.x,
                        robot.y,
                        robot.theta
                    );
                }
            );
    }

private:
    Robot robot;

    double v;
    double omega;

    rclcpp::Subscription<
        geometry_msgs::msg::Twist
    >::SharedPtr subscription_;

    rclcpp::Publisher<
        geometry_msgs::msg::Pose2D
    >::SharedPtr pose_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<RobotNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}