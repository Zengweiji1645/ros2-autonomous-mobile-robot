#include "my_robot/Controller.h"
#include <cmath>


Controller::Controller(
    double kp_distance,
    double ki_distance,
    double kd_distance,
    double kp_angle,
    double ki_angle,
    double kd_angle
)
: distance_pid(kp_distance, ki_distance, kd_distance),
  angle_pid(kp_angle, ki_angle, kd_angle)
{
}
 



void Controller::calculate(
    double target_x,
    double target_y,
    double current_x,
    double current_y,
    double current_theta,
    double &v,
    double &omega,
    double dt
)
{

    double dx = target_x - current_x;

    double dy = target_y - current_y;


    double distance = sqrt(
        dx*dx + dy*dy
    );


    double target_angle = atan2(
        dy,
        dx
    );


    double angle_error =
        target_angle - current_theta;

while (angle_error > M_PI)
{
    angle_error -= 2 * M_PI;
}

while (angle_error < -M_PI)
{
    angle_error += 2 * M_PI;
}

    v = distance_pid.calculate(
        distance,
        0.0,
        dt
    );


    omega = angle_pid.calculate(
        angle_error,
        0.0,
        dt
    );

}
void Controller::reset()
{
    distance_pid.reset();
    angle_pid.reset();
}