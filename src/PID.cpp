#include <algorithm>
#include "PID.h"
#include <chrono>
#include <iostream>

using namespace std;
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kprop, double Kint, double Kderiv) {
    Kp = Kprop;
    Ki = Kint;
    Kd = Kderiv;

    integral = 0;
    last_error = 0;
    last_timestamp = 0;
    timestamp = 0;
    reference = 0.0;
    error = 0.0;

    delta_K = {0.01, 0.01, 0.01};
}

void PID::UpdateError(double cte) {
    last_timestamp = timestamp;
    timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);

    error = cte - reference;

    double delta_T = (timestamp - last_timestamp) * 1e-3;
    std::cout << "Delta T: " << delta_T << std::endl;

    p_error = - Kp * error;
    if (last_timestamp != 0){
        integral += error;
        i_error = - Ki * integral * delta_T;

        d_error = - Kd * (error - last_error) / delta_T;
        last_error = error;
    }
    else{
        integral += error;
        i_error = 0.0;

        d_error = 0.0;
        last_error = error;
    }
    return;
}

double PID::TotalError() {
    double control = p_error + i_error + d_error;

    if (control > 0)
        control = min(1.0, control);
    else
        control = max(-1.0, control);

    return control;
}
