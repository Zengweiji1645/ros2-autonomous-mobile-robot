#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include "my_robot/Controller.h"
#include "my_robot/AStarPlanner.h"
#include <visualization_msgs/msg/marker.hpp>
#include <stdexcept>

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode(): Node("controller_node"),controller
    (
    0.5, 0.05, 0.02,
    2.0, 0.05, 0.3
    )
    {
        path = planner.plan(0,0,4,4);
        if (path.size() < 2)
        {
            throw std::runtime_error("A* failed to find a valid path");
        }
        current_waypoint_index = 1;
        target_x =path[current_waypoint_index].first;
        target_y =path[current_waypoint_index].second;

        cmd_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

        obstacle_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("obstacles", rclcpp::QoS(1).transient_local());
       
        path_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>( "planned_path",rclcpp::QoS(1).transient_local());
        
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = this->get_clock()->now();
        path_marker.ns = "planned_path";
        path_marker.id = 0;
        path_marker.type =visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action =visualization_msgs::msg::Marker::ADD; 
        path_marker.pose.orientation.x = 0.0;
        path_marker.pose.orientation.y = 0.0;
        path_marker.pose.orientation.z = 0.0;
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x = 0.15;
        path_marker.color.r = 1.0;
        path_marker.color.g = 0.0;
        path_marker.color.b = 0.0;
        path_marker.color.a = 0.5;

        visualization_msgs::msg::Marker obstacle_marker;
        obstacle_marker.header.frame_id = "map";
        obstacle_marker.header.stamp = this->get_clock()->now();
        obstacle_marker.ns = "obstacles";
        obstacle_marker.id = 0;
        obstacle_marker.type =visualization_msgs::msg::Marker::CUBE_LIST;
        obstacle_marker.action =visualization_msgs::msg::Marker::ADD;
        obstacle_marker.scale.x = 0.8;
        obstacle_marker.scale.y = 0.8;
        obstacle_marker.scale.z = 0.5;
        obstacle_marker.pose.orientation.w = 1.0;
        obstacle_marker.color.r = 0.0;
        obstacle_marker.color.g = 0.0;
        obstacle_marker.color.b = 1.0;
        obstacle_marker.color.a = 1.0;

        const auto & grid = planner.getGrid();
        for (std::size_t x = 0; x < grid.size(); x++)
        {
            for (std::size_t y = 0; y < grid[x].size(); y++)
            {
                if (grid[x][y] == 1)
                {
                    geometry_msgs::msg::Point p;
                    p.x = x;
                    p.y = y;
                    p.z = 0.25;
                    obstacle_marker.points.push_back(p);
                }
            }
        }
        obstacle_publisher_->publish(obstacle_marker);


        for (const auto & point : path)
        {
            geometry_msgs::msg::Point p;

            p.x = point.first;
            p.y = point.second;
            p.z = 0.05;

            path_marker.points.push_back(p);
        }
        path_publisher_->publish(path_marker);

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
                        controller.reset();// 后面还有 waypoint，每到一个新的 waypoint，我们清掉上一个 waypoint 留下来的 PID 积分和历史误差，再开始追踪新目标
                         if (current_waypoint_index + 1 < path.size())
                         {
                             current_waypoint_index++;

                            target_x =
                                path[current_waypoint_index].first;

                            target_y =
                                path[current_waypoint_index].second;

                            RCLCPP_INFO(
                                this->get_logger(),
                                "Next waypoint: (%.1f, %.1f)",
                                target_x,
                                target_y
                            );

                            return;// 继续追踪下一个 waypoint
                        }

                        // 已经是最后一个 waypoint
                        if (!target_reached)
                        {
                            target_reached = true;

                            RCLCPP_INFO(
                                this->get_logger(),
                                "Final target reached! distance=%.4f",
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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_subscription_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_publisher_;

    AStarPlanner planner;
    std::vector<std::pair<int, int>> path;

    std::size_t current_waypoint_index;
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