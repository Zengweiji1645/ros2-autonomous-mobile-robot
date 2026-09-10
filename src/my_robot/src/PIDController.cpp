#include "my_robot/PIDController.h"


PIDController::PIDController(
    double kp_,
    double ki_,
    double kd_
)
{

    kp = kp_;
    ki = ki_;
    kd = kd_;

    integral = 0.0;

    previous_error = 0.0;
    integral_limit = 1.0; // Set a reasonable limit for the integral term

}

double PIDController::calculate(
    double target,
    double current,
    double dt
)
{

    double error = target - current;


    integral += error * dt;
    if (integral > integral_limit)
    {
        integral = integral_limit;
    }
    else if (integral < -integral_limit)
    {
        integral = -integral_limit;
    }

if (dt <= 0.0)
{
    return 0.0;
}
    double derivative =
        (error - previous_error) / dt;


    previous_error = error;


    double output =
        kp * error
        +
        ki * integral
        +
        kd * derivative;


    return output;

}
void PIDController::reset()
{
    integral = 0.0;
    previous_error = 0.0;
}
