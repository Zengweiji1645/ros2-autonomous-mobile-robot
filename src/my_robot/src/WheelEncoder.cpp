#include "my_robot/WheelEncoder.h"
#include <cmath>

// 构造函数，初始化轮子半径和每圈的编码器脉冲数
WheelEncoder::WheelEncoder(double wheel_radius, int ticks_per_revolution)
 {
    wheel_radius_ = wheel_radius;// 初始化轮子半径
    ticks_per_revolution_ = ticks_per_revolution;// 初始化每圈的编码器脉冲数

    accumlated_ticks_ = 0.0; // 初始化累计的编码器脉冲数为 0
    reported_ticks_ = 0; // 初始化输出的编码器脉冲数为
 }

// 将编码器脉冲数转换为距离
 double WheelEncoder::ticksToDistance(int ticks)const
 {
    // 计算轮子的周长
    double circumference = 2 * M_PI * wheel_radius_;
    // 计算编码器脉冲数对应的轮子转动圈数
    double revolutions = static_cast<double>(ticks) / ticks_per_revolution_;
    // 将转动圈数转换为距离
    double distance = revolutions * circumference;
    return distance;
 }

 // 更新累计的编码器脉冲数
 int WheelEncoder::updateTicks(double distance)
 {
    double circumference = 2 * M_PI * wheel_radius_;
    double delta_ticks = distance / circumference * ticks_per_revolution_;

    accumlated_ticks_ += delta_ticks;

    int new_reported_ticks = static_cast<int>(round(accumlated_ticks_));
    
    int ticks_change = new_reported_ticks - reported_ticks_;
    
    reported_ticks_ = new_reported_ticks;
    
    return ticks_change;
 }