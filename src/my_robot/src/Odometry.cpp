#include "my_robot/Odometry.h"
#include <cmath>

Odometry::Odometry(
    double x_,
    double y_,
    double theta_
)
{
    x = x_;
    y = y_;
    theta = theta_;
}

void Odometry::update(
    double distance_left,// 左轮行驶的距离
    double distance_right,// 右轮行驶的距离 
    double wheel_base// 轮距
)
{
    // 计算机器人前进的距离和旋转的角度
    double distance = (distance_left + distance_right) / 2.0;
    double delta_theta = (distance_right - distance_left) / wheel_base;
    // 计算起点与终点的平均角度，用于更新位置    
    double middle_theta = theta + delta_theta/2.0;
    x += distance * std::cos(middle_theta);
    y += distance * std::sin(middle_theta);
    theta += delta_theta;
    // 将角度限制在 -π 到 π 之间（即角度归一化）
    while (theta > M_PI)
    {
    theta -= 2.0 * M_PI;
    }

    while (theta < -M_PI)
    {
        theta += 2.0 * M_PI;
    }
}