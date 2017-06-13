#include <algorithm>
#include "PID.h"
#include <chrono>
#include <iostream>

using namespace std;
/*
* TODO: Complete the PID class.
*/

PID::PID() {
    K = {0.09, 0.0045, 0.05};

    delta_K = {K[0] * 0.1, K[1] * 0.1, K[2] * 0.1, 0.0};    // Last parameter means that a coefficient is being adjusted
    //delta_K = {0.0, 0.0, 0.0, 0.0};

    error_squared = {1E100, 0.0};
    twiddle_parameter = 0;
    twiddle_tries = 0;
    iteration = 0;
}

PID::~PID() {}

void PID::Init(){
    last_error = 0;
    timestamp = {0, 0};
    control = 0.0;
    error_squared[1] = 0.0;
}

void PID::UpdateError(double cte) {

    // Get timestamp
    timestamp = {timestamp[1], std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1)};
    double delta_T = (timestamp[1] - timestamp[0]) * 1e-3;

    /*reference.push_back(cte);
    if (sizeof(reference) > 10)
        reference.pop_back();
    cte = accumulate( reference.begin(), reference.end(), 0.0)/reference.size();*/

    // Proportional error
    p_error = cte; // - reference;

    // Integral error
    i_error += p_error * delta_T;

    // Derivative error
    d_error = (p_error - last_error) / delta_T;
    last_error = p_error;

    // Error squared
    error_squared[1] += control * control * p_error * p_error;

    // Avoid division by zero
    if (timestamp[0] == 0){
        i_error = 0.0;
        d_error = 0.0;
    }

    return;
}

double PID::GetControl() {
    control = - K[0] * p_error - K[1] * i_error - K[2] * d_error;

    if (control > 0)
        control = min(1.0, control);
    else
        control = max(-1.0, control);

    return control;
}

void PID::UpdateTwiddle() {

    switch (int(delta_K[3])) {
    case 1:
        if (error_squared[1] < error_squared[0]) {
            error_squared[0] = error_squared[1];
            delta_K[twiddle_parameter] *= 1.1;
            delta_K[3] = 0;
            twiddle_parameter = (twiddle_parameter + 1) % 3;
            twiddle_tries = 0;
        } else {
            K[twiddle_parameter] -= 2 * delta_K[twiddle_parameter];
            delta_K[3] = 2;
            twiddle_tries ++;
        }
        break;
    case 2:
        if (error_squared[1] < error_squared[0]) {
            error_squared[0] = error_squared[1];
            delta_K[twiddle_parameter] *= 1.1;
            delta_K[3] = 0;
            twiddle_parameter = (twiddle_parameter + 1) % 3;
            twiddle_tries = 0;
        } else {
            K[twiddle_parameter] += delta_K[twiddle_parameter];
            delta_K[twiddle_parameter] *= 0.9;
            delta_K[3] = 0;
            twiddle_parameter = (twiddle_parameter + 1) % 3;
            twiddle_tries ++;
        }
        break;
    }

    if (twiddle_tries == 12){
        error_squared[0] = error_squared[1];
        twiddle_tries = 0;
    }
    else if (int(delta_K[3]) == 0){
        K[twiddle_parameter] += delta_K[twiddle_parameter];
        delta_K[3] = 1;
    }

    Init();
    iteration ++;

    return;
}
