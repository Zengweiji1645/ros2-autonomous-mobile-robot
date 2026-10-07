#ifndef LIDAR_H
#define LIDAR_H
#include <vector>
#include "my_robot/World.h"

class Lidar
{
public:
    // 计算机器人到障碍物的距离
    double calculateDistance(
        double robot_x,
        double robot_y,
        double obstacle_x,
        double obstacle_y
    ) const;

    // 计算障碍物相对于机器人正前方的角度
    double calculateRelativeAngle(
        double robot_x,
        double robot_y,
        double robot_theta,
        double obstacle_x,
        double obstacle_y
    ) const;

    // 模拟激光雷达的射线投射
    double castRay(
        double robot_x,
        double robot_y,
        double robot_theta,
        double relative_angle,
        const World & world
    ) const;

    // 扫描环境，返回每个角度的距离测量值
    std::vector<double> scan(
        double robot_x,
        double robot_y,
        double robot_theta,
        const World & world
    ) const;
};

#endif