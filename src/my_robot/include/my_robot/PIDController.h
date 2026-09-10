#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H


class PIDController
{

public:

    PIDController(
        double kp_,
        double ki_,
        double kd_
    );


    double calculate(
        double target,
        double current,
        double dt
    );

    void reset();
    
private:
    double kp;
    double ki;
    double kd;


    double integral;
    double previous_error;
    double integral_limit; // Limit for the integral term to prevent windup


};


#endif