#ifndef BBOT_BALANCE_CONTROLLER_PID_H
#define BBOT_BALANCE_CONTROLLER_PID_H

class PIDController
{
public:
    float P;
    float I;
    float D;

    float output_ramp;
    float limit;

    float error_prev;
    float output_prev;
    float integral_prev;
    float derivative_prev;

    PIDController(float p, float i, float d, float ramp, float lim);

    float operator()(float error, float dt);

    void reset();
};

#endif