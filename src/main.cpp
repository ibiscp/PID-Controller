#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <chrono>
#include <fstream>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
    return M_PI;
}
double deg2rad(double x) {
    return x * pi() / 180;
}
double rad2deg(double x) {
    return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main() {
    uWS::Hub h;

    // TODO: Initialize the pid
    PID pid;

    // Create file to save history
    std::ofstream myfile ("..\result.csv");
    myfile << "Iteration, " << "Twiddle Parameter, " << \
           "K0, " << "K1, " << "K2, " << \
           "Delta K0, " << "Delta K1, " << "Delta K2, " << \
           "Twiddle Tries, " << \
           "Last Error, " << "Minimum Error\n";

    // Interval of each round
    int interval = 60000;

    long long int oldTime = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1) - interval;
    long long int newTime;
    std::vector<string> controller = {"Proportional", "Integral", "Derivative"};

    h.onMessage([&pid, &oldTime, &newTime, &controller, &myfile, &interval](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {

                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    double steer_value;
                    /*
                    * TODO: Calculate steering value here, remember the steering value is [-1, 1].
                    * NOTE: Feel free to play around with the throttle and speed. Maybe use
                    * another PID controller to control the speed!
                    */

                    // Calculate Steering
                    newTime = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);

                    if (newTime - oldTime >= interval && (pid.delta_K[0] + pid.delta_K[1] + pid.delta_K[2]) > 1E-6) {

                        if (pid.error_squared[1] > 1E-200) {
                            std::cout << "\nLast Error:\t" << pid.error_squared[1] << std::endl;
                            std::cout << "Minimum Error:\t" << min(pid.error_squared[0],pid.error_squared[1]) << std::endl;
                            myfile << pid.error_squared[1] << ", " << pid.error_squared[0] << "\n";
                        }

                        pid.UpdateTwiddle();

                        std::cout << "\nIteration: " << pid.iteration << "\t" << controller[pid.twiddle_parameter] << "\n" << std::scientific;
                        std::cout << "K:\t\t[" << pid.K[0] << ", " << pid.K[1] << ", " << pid.K[2] << "]\n";
                        std::cout << "Delta K:\t[" << pid.delta_K[0] << ", " << pid.delta_K[1] << ", " << pid.delta_K[2] << "]" << std::endl;
                        std::cout << "Twiddle tries:\t" << pid.twiddle_tries << std::scientific;

                        myfile << pid.iteration << ", " << controller[pid.twiddle_parameter] << ", " << \
                               pid.K[0] << ", " << pid.K[1] << ", " << pid.K[2] << ", " << \
                               pid.delta_K[0] << ", " << pid.delta_K[1] << ", " << pid.delta_K[2] << ", " << \
                               pid.twiddle_tries << ", ";

                        oldTime = newTime;
                    }

                    pid.UpdateError(cte);
                    steer_value = pid.GetControl();

                    // DEBUG
                    //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Tmestamp: " << (oldTime - newTime) << std::endl;

                    // Calculate Throttle
                    double throttle;
                    double throttle_desired = 0.4;// - abs(steer_value) * 0.2;
                    if (speed < throttle_desired*100*0.8)
                        throttle = 1.0;
                    else if (speed > throttle_desired * 100 * 1.2)
                        throttle = -1.0;
                    else
                        throttle = throttle_desired;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
