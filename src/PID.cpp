#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    integral = 0;
    last_cte = 0;
}

void PID::UpdateError(double cte) {
    p_error = - Kp * cte;

    integral += cte;
    i_error = - Ki * integral;

    d_error = - Kd * (cte - last_cte);
    last_cte = cte;
}

double PID::TotalError() {
    return (p_error + i_error + d_error);
}
