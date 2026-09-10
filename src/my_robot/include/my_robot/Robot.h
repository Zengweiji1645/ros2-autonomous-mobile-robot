#ifndef ROBOT_H
#define ROBOT_H

class Robot
{
public:
    double x;
    double y;
    double theta;

    Robot(double x0, double y0, double theta0);

    void update(
        double v,
        double omega,
        double dt
    );
};

#endif
