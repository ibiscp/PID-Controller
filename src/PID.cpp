#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    integral = 0;
    last_cte = 0;
}

void PID::UpdateError(double cte) {
    double proportional = - Kp * cte;

    integral += cte;
    double integral = - Ki * integral;

    double derivative = - Kd * (cte - last_cte);
    last_cte = cte
}

double PID::TotalError() {
    return 0.4;
}
