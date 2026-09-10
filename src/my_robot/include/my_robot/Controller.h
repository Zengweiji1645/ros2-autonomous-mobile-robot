#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "my_robot/PIDController.h"

class Controller
{

public:


    Controller(
        double kp_distance,
        double ki_distance,
        double kd_distance,
        double kp_angle,
        double ki_angle,
        double kd_angle
    );


    void calculate(
        double target_x,
        double target_y,
        double current_x,
        double current_y,
        double current_theta,
        double &v,
        double &omega,
        double dt
    );
    void reset();
    
    private:

    PIDController distance_pid;
    PIDController angle_pid;

};


#endif