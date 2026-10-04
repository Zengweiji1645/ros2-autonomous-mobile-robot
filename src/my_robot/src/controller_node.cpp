#include <cmath>
#include <stdexcept>
#include "my_robot/Controller.h"
#include "my_robot/AStarPlanner.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>

class ControllerNode : public rclcpp::Node
{
public:

    // 构造函数，初始化 PID 控制器和 A* 路径规划器
    ControllerNode(): Node("controller_node"),controller
    (
    0.5, 0.05, 0.02,
    2.0, 0.05, 0.3
    )
    {
        // 使用 A* 算法规划路径
        path = planner.plan(0,0,4,4);
        if (path.size() < 2)
        {
            throw std::runtime_error("A* failed to find a valid path");
        }
        current_waypoint_index = 1;
        target_x =path[current_waypoint_index].first;
        target_y =path[current_waypoint_index].second;

        // 发布速度指令
        cmd_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

        // 发布障碍物和路径的可视化信息
        obstacle_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("obstacles", rclcpp::QoS(1).transient_local());
       
        // 发布规划路径的可视化信息
        path_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>( "planned_path",rclcpp::QoS(1).transient_local());
        
        // 发布障碍物和路径的可视化信息
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
       
        // 发布障碍物和路径的可视化信息
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
       
        // 获取网格地图并发布障碍物信息
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

        // 发布路径信息
        for (const auto & point : path)
        {
            geometry_msgs::msg::Point p;

            p.x = point.first;
            p.y = point.second;
            p.z = 0.05;

            path_marker.points.push_back(p);
        }
        path_publisher_->publish(path_marker);

        // 订阅机器人当前位置
        odom_subscription_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "odom",
                10,
                [this](const nav_msgs::msg::Odometry & odom)
                {
                    double current_x = odom.pose.pose.position.x;
                    double current_y = odom.pose.pose.position.y;
                   // 将四元数转换为欧拉角，获取当前朝向
                    tf2::Quaternion q(
                        odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w
                    );
                    // 将四元数转换为欧拉角，获取当前朝向
                    double roll, pitch, current_theta;
                    tf2::Matrix3x3(q).getRPY(roll, pitch, current_theta);
                    // 计算机器人当前位置与目标位置之间的距离和角度误差，并使用 PID 控制器计算线速度和角速度
                    double v, omega;
                    double dx = target_x - current_x;
                    double dy = target_y - current_y;
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

                        // 停止机器人
                        geometry_msgs::msg::Twist cmd;

                        cmd.linear.x = 0.0;
                        cmd.angular.z = 0.0;
                        cmd_publisher_->publish(cmd);
                        return;
                    }

                    // 计算线速度和角速度
                    controller.calculate(
                        target_x,
                        target_y,
                        current_x,
                        current_y,
                        current_theta,
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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    
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