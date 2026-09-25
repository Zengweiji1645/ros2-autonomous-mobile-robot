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
    previous_v = 0.0;
    previous_omega = 0.0;
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

    double heading_factor = std::cos(angle_error);
    if (heading_factor < 0.0) {
        heading_factor = 0.0;
    }
    v *= heading_factor;
    double max_v = 1.0; // Maximum speed limit 

    if (v > max_v) {
        v = max_v;
    }

    if (v < -max_v) {
        v = -max_v;
    }

    double max_omega = 1.5; // Maximum angular speed limit

    if (omega > max_omega) {
        omega = max_omega;
    }

    if (omega < -max_omega) {
        omega = -max_omega;
    }

    double max_linear_acceleration = 1.0; // Maximum acceleration limit
    double max_angular_acceleration = 2.0; // Maximum angular acceleration limit
    double acceleration = (v - previous_v) / dt;
    double angular_acceleration = (omega - previous_omega) / dt;

    if (acceleration > max_linear_acceleration) {
        v = previous_v + max_linear_acceleration * dt;
    }

    if (acceleration < -max_linear_acceleration) {
        v = previous_v - max_linear_acceleration * dt;
    }

    if (angular_acceleration > max_angular_acceleration) {
        omega = previous_omega + max_angular_acceleration * dt;
    }

    if (angular_acceleration < -max_angular_acceleration) {
        omega = previous_omega - max_angular_acceleration * dt;
    }

    previous_v = v;
    previous_omega = omega;
}
void Controller::reset()
{
    distance_pid.reset();
    angle_pid.reset();
}