#include "my_robot/Lidar.h"
#include <cmath>

double Lidar::calculateDistance(
    double robot_x,
    double robot_y,
    double obstacle_x,
    double obstacle_y
) const
{
    double dx = obstacle_x - robot_x;
    double dy = obstacle_y - robot_y;
    return std::sqrt(dx * dx + dy * dy);
}

double Lidar::calculateRelativeAngle(
    double robot_x,
    double robot_y,
    double robot_theta,
    double obstacle_x,
    double obstacle_y
) const
{
    double dx = obstacle_x - robot_x;
    double dy = obstacle_y - robot_y;
    
    double global_angle = std::atan2(dy, dx);//全局视角（map）下的障碍物方位
    double relative_angle = global_angle - robot_theta;//机器人视角下（baselink）的障碍物方位
    
    while (relative_angle > M_PI)
    {
        relative_angle -= 2 * M_PI;
    }
    while (relative_angle < -M_PI)
    {
        relative_angle += 2 * M_PI;
    }

    return relative_angle;
}

// 模拟激光雷达的射线投射(碰到障碍物就返回当前距离，没碰到就返回最大射线长度)
double Lidar::castRay(
    double robot_x,
    double robot_y,
    double robot_theta,
    double relative_angle,
    const World & world
) const
{
    double global_angle = robot_theta + relative_angle;
    double step = 0.02; // 步长
    double max_range = 6.0; // 最大射线长度

    for (double distance = 0.0; distance <= max_range; distance += step)
    {
        double scan_x = robot_x + distance * std::cos(global_angle);
        double scan_y = robot_y + distance * std::sin(global_angle);

        if (world.isObstacle(scan_x, scan_y)) // 使用World类的isObstacle方法
        {
            return distance; // 射线碰到障碍物，返回当前距离
        }
    }

    return max_range; // 没有碰到障碍物，返回最大射线长度
}

// 扫描环境，返回每个角度的距离测量值
std::vector<double> Lidar::scan(
    double robot_x,
    double robot_y,
    double robot_theta,
    const World & world
) const
{
    std::vector<double> ranges;
    double angle_increment = M_PI / 180.0; // 每1度扫描一次
    double angle_max = M_PI; // 最大扫描角度为180度
    double angle_min = -M_PI; // 最小扫描角度为-180度
    for (double angle = angle_min; angle <= angle_max; angle += angle_increment) // 每度扫描一次
    {
        double distance = castRay(
            robot_x,
            robot_y,
            robot_theta,
            angle,
            world
        );
        ranges.push_back(distance);
    }
    return ranges;
}