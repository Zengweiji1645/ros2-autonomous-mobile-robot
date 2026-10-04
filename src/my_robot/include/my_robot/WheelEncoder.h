#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

class WheelEncoder {
public:
    // 构造函数，初始化轮子半径和每圈的编码器脉冲数
    WheelEncoder(double wheel_radius, int ticks_per_revolution);
    // 将编码器脉冲数转换为距离
    double ticksToDistance(int ticks)const;
    // 更新累计的编码器脉冲数
    int updateTicks(double distance);

private:
    double wheel_radius_;
    int ticks_per_revolution_;

    double accumlated_ticks_; // 累计的编码器脉冲数
    int reported_ticks_; //输出的编码器脉冲数
};

#endif