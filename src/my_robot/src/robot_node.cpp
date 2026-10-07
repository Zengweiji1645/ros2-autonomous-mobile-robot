#include <vector>
#include <cmath>
#include "my_robot/Robot.h"
#include "my_robot/Odometry.h"
#include "my_robot/WheelEncoder.h"
#include "my_robot/Lidar.h"
#include "my_robot/World.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class RobotNode : public rclcpp::Node
{
public:
    RobotNode()// 构造函数
        : Node("robot_node"),// 初始化节点名称为 "robot_node"
          robot(0.0, 0.0, 0.0),// 初始化机器人的x, y, theta
          odometry(0.0, 0.0, 0.0),// 初始化机器人的里程计信息x, y, theta
          left_encoder_(0.1, 1000), // 假设轮子半径为 0.1 米，每圈编码器脉冲数为 1000
          right_encoder_(0.1, 1000), // 假设轮子半径为 0.1 米，每圈编码器脉冲数为 1000
          v(0.0),// 初始化线速度为 0.0
          omega(0.0)// 初始化角速度为 0.0
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
       
        //发布机器人里程计信息
        odom_publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odom",10);

        // 发布机器人里程计轨迹
        odom_trajectory_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("odom_trajectory",10);
       
        // 发布激光雷达扫描数据
        scan_publisher_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("scan",10);
        
        // 创建 TF 广播器    
        tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(this);
        // 创建静态 TF 广播器
        static_tf_broadcaster_ =std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        // 发布 base_link 到 laser_frame 的静态变换
        geometry_msgs::msg::TransformStamped base_link_to_laser;
        base_link_to_laser.header.stamp = this->get_clock()->now();
        base_link_to_laser.header.frame_id = "base_link";
        base_link_to_laser.child_frame_id = "laser_frame";

        base_link_to_laser.transform.translation.x = 0.0;
        base_link_to_laser.transform.translation.y = 0.0;
        base_link_to_laser.transform.translation.z = 0.0;

        base_link_to_laser.transform.rotation.x = 0.0;
        base_link_to_laser.transform.rotation.y = 0.0;
        base_link_to_laser.transform.rotation.z = 0.0;
        base_link_to_laser.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(base_link_to_laser);

        // 每 0.1 秒更新一次机器人
        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]()
                {
                    double dt = 0.1;

                    auto current_time = this->get_clock()->now();
                    //用现在的速度和角速度更新机器人位置和朝向
                    robot.update(v, omega, dt);

                    // 发布激光雷达扫描数据
                    std::vector<double> ranges = lidar_.scan(
                        robot.x,
                        robot.y,
                        robot.theta,
                        world_
                    );

                    //根据轮距和左右轮的速度计算里程计信息（更新x，y，theta）
                    double wheel_base = 0.4; // 假设轮距为 0.4 米
                    double velocity_left = v - omega * wheel_base / 2.0;
                    double velocity_right = v + omega * wheel_base / 2.0;
                    double distance_left = velocity_left * dt;
                    double distance_right = velocity_right * dt;
                    int left_ticks = left_encoder_.updateTicks(distance_left);
                    int right_ticks = right_encoder_.updateTicks(distance_right);
                    double mesured_distance_left = left_encoder_.ticksToDistance(left_ticks);
                    double mesured_distance_right = right_encoder_.ticksToDistance(right_ticks);
                    odometry.update(mesured_distance_left, mesured_distance_right, wheel_base);

                    // 发布激光雷达扫描数据
                    sensor_msgs::msg::LaserScan scan_msg;
                    scan_msg.header.stamp = current_time;
                    scan_msg.header.frame_id = "laser_frame";
                    scan_msg.angle_min = -M_PI ;
                    scan_msg.angle_max = M_PI ;
                    scan_msg.angle_increment = M_PI / 180.0; // 每1度扫描一次
                    scan_msg.range_min = 0.0;
                    scan_msg.range_max = 6.0;
                    scan_msg.ranges.assign(ranges.begin(), ranges.end());
                    scan_publisher_->publish(scan_msg);

                    // 发布里程计信息
                    nav_msgs::msg::Odometry odom_msg;
                    odom_msg.header.stamp = current_time;//时间戳，告诉别人这条里程计数据产生的时间
                    //这条 Odometry 消息描述的是 base_link 相对于 odom 的状态
                    odom_msg.header.frame_id = "odom";
                    odom_msg.child_frame_id = "base_link";
                    //分成两个pose，第一个pose是保存了位姿和不确定性，第二个pose是位姿
                    odom_msg.pose.pose.position.x = odometry.x;
                    odom_msg.pose.pose.position.y = odometry.y;
                    odom_msg.pose.pose.position.z = 0.0;
                    odom_msg.pose.pose.orientation.x = 0.0;
                    odom_msg.pose.pose.orientation.y = 0.0;
                    odom_msg.pose.pose.orientation.z =std::sin(odometry.theta / 2.0);
                    odom_msg.pose.pose.orientation.w =std::cos(odometry.theta / 2.0);
                    //twist是速度，分成两个twist，第一个twist是保存了速度和不确定性，第二个twist是速度
                    odom_msg.twist.twist.linear.x = v;
                    odom_msg.twist.twist.angular.z = omega;
                    odom_publisher_->publish(odom_msg);

                    // 发布机器人在世界坐标系中的位置和朝向
                    geometry_msgs::msg::TransformStamped transform;
                    transform.header.stamp = current_time;
                    transform.header.frame_id = "odom";
                    transform.child_frame_id = "base_link";
                    transform.transform.translation.x = odometry.x;
                    transform.transform.translation.y = odometry.y;
                    transform.transform.translation.z = 0.0;
                    transform.transform.rotation.x = 0.0;
                    transform.transform.rotation.y = 0.0;
                    transform.transform.rotation.z = std::sin(odometry.theta / 2.0);
                    transform.transform.rotation.w = std::cos(odometry.theta / 2.0);
                    tf_broadcaster_->sendTransform(transform);

                    // 发布机器人当前位置和朝向以进行控制
                    geometry_msgs::msg::Pose2D pose;
                    pose.x = robot.x;
                    pose.y = robot.y;
                    pose.theta = robot.theta;
                    pose_publisher_->publish(pose);

                    // 发布机器人当前位置和朝向的 Marker 给 RViz
                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "map";
                    marker.header.stamp = current_time;
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

                    // 将当前里程计位置添加到里程计轨迹点列表中
                    geometry_msgs::msg::Point odom_trajectory_point;
                    odom_trajectory_point.x = odometry.x;
                    odom_trajectory_point.y = odometry.y;
                    odom_trajectory_point.z = 0.06;
                    odom_trajectory_points_.push_back(odom_trajectory_point);

                    // 发布轨迹
                    visualization_msgs::msg::Marker trajectory_marker;
                    trajectory_marker.header.frame_id = "map";
                    trajectory_marker.header.stamp = current_time;
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

                    // 发布里程计轨迹
                    visualization_msgs::msg::Marker odom_trajectory_marker;
                    odom_trajectory_marker.header.frame_id = "odom";
                    odom_trajectory_marker.header.stamp = current_time;
                    odom_trajectory_marker.ns = "odom_trajectory";
                    odom_trajectory_marker.id = 0;
                    odom_trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                    odom_trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
                    odom_trajectory_marker.scale.x = 0.05;
                    odom_trajectory_marker.color.a = 1.0;
                    odom_trajectory_marker.color.r = 1.0;
                    odom_trajectory_marker.color.g = 1.0;
                    odom_trajectory_marker.color.b = 0.0;
                    odom_trajectory_marker.pose.orientation.w = 1.0;
                    odom_trajectory_marker.points = odom_trajectory_points_;
                    odom_trajectory_publisher_->publish(odom_trajectory_marker);

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
    Odometry odometry;
    WheelEncoder left_encoder_;
    WheelEncoder right_encoder_;
    World world_; // 创建 World 对象，用于表示环境中的障碍物
    Lidar lidar_;
    double v;
    double omega;
   
    // 订阅 Controller 发来的速度
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
   
    // 发布机器人当前位置和朝向给 Controller
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;
   
    // 发布机器人当前位置和朝向的 Marker给RViz
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
   
    // 发布机器人里程计信息给 Controller
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
   
    // 定时器，用于定期更新机器人状态
    rclcpp::TimerBase::SharedPtr timer_;
   
    // 记录机器人实际走过的路线点
    std::vector<geometry_msgs::msg::Point> trajectory_points_;
   
    // 发布机器人实际走过的路线给RViz
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_publisher_;
   
    // 记录机器人里程计轨迹点
    std::vector<geometry_msgs::msg::Point>odom_trajectory_points_;
   
    // 发布机器人里程计轨迹给RViz
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr odom_trajectory_publisher_;
   
    // TF 广播器，用于发布机器人在世界坐标系中的位置和朝向
    std::unique_ptr<tf2_ros::TransformBroadcaster>tf_broadcaster_;
   
    // 静态 TF 广播器，用于发布 map 到 odom 的静态变换    
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster>static_tf_broadcaster_;
   
    // 发布激光雷达扫描数据
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =std::make_shared<RobotNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}