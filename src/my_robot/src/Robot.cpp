#include "my_robot/Robot.h"
#include <cmath>

Robot::Robot(
    double x0,
    double y0,
    double theta0
)
{
    x = x0;
    y = y0;
    theta = theta0;
}

void Robot::update(
    double v,
    double omega,
    double dt
)
{
    x += v * std::cos(theta) * dt;
    y += v * std::sin(theta) * dt;
    theta += omega * dt;
}
