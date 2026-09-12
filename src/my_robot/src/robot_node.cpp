#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "my_robot/Robot.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <cmath>

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
            this->create_publisher<geometry_msgs::msg::Pose2D>("robot_pose",10);
        // 发布机器人当前的位置和朝向（不记录机器人的行动路线）
        marker_publisher_ =
            this->create_publisher<visualization_msgs::msg::Marker>("robot_marker",10);
        //发布机器人实际走过的路线
        trajectory_publisher_ =
            this->create_publisher<visualization_msgs::msg::Marker>("actual_trajectory",10);

        tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // 每 0.1 秒更新一次机器人
        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]()
                {
                    double dt = 0.1;

                    robot.update(v, omega, dt);

                    geometry_msgs::msg::TransformStamped transform;
                    transform.header.stamp =
                        this->get_clock()->now();
                    transform.header.frame_id = "map";
                    transform.child_frame_id = "base_link";
                    transform.transform.translation.x = robot.x;
                    transform.transform.translation.y = robot.y;
                    transform.transform.translation.z = 0.0;
                    transform.transform.rotation.x = 0.0;
                    transform.transform.rotation.y = 0.0;
                    transform.transform.rotation.z = std::sin(robot.theta / 2.0);
                    transform.transform.rotation.w = std::cos(robot.theta / 2.0);
                    tf_broadcaster_->sendTransform(transform);

                    geometry_msgs::msg::Pose2D pose;

                    pose.x = robot.x;
                    pose.y = robot.y;
                    pose.theta = robot.theta;

                    pose_publisher_->publish(pose);
                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "map";
                    marker.header.stamp = this->get_clock()->now();

                    marker.ns = "robot";
                    marker.id = 0;

                    marker.type = visualization_msgs::msg::Marker::CYLINDER;
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    marker.pose.position.x = robot.x;
                    marker.pose.position.y = robot.y;
                    marker.pose.position.z = 0.1;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.4;
                    marker.scale.y = 0.4;
                    marker.scale.z = 0.2;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 0.5;
                    marker.color.b = 1.0;
                    marker_publisher_->publish(marker);

                    // 将当前机器人位置添加到轨迹点列表中
                    geometry_msgs::msg::Point trajectory_point;
                    trajectory_point.x = robot.x;
                    trajectory_point.y = robot.y;
                    trajectory_point.z = 0.03;
                    trajectory_points_.push_back(trajectory_point);

                    // 发布轨迹
                    visualization_msgs::msg::Marker trajectory_marker;
                    trajectory_marker.header.frame_id = "map";
                    trajectory_marker.header.stamp = this->get_clock()->now();
                    trajectory_marker.ns = "actual_trajectory";
                    trajectory_marker.id = 0;
                    trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                    trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
                    trajectory_marker.scale.x = 0.08;
                    trajectory_marker.color.a = 1.0;
                    trajectory_marker.color.r = 0.0;
                    trajectory_marker.color.g = 1.0;
                    trajectory_marker.color.b = 0.0;
                    trajectory_marker.pose.orientation.w = 1.0;
                    trajectory_marker.points = trajectory_points_;
                    trajectory_publisher_->publish(trajectory_marker);

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

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Point> trajectory_points_;

    std::unique_ptr<tf2_ros::TransformBroadcaster>tf_broadcaster_;
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