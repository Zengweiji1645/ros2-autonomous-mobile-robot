#ifndef ODOMETRY_H
#define ODOMETRY_H

class Odometry
{
public:
    Odometry(
        double x_,
        double y_,
        double theta_
    );

    void update(
        double distance_left,
        double distance_right,
        double wheel_base
    );

    double x;
    double y;
    double theta;
};

#endif