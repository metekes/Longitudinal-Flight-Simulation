#include "PD_controller.hpp"

PD_controller::PD_controller(){
    // initialize integral errors
    integral_err_altitude = 0;
    integral_err_velocity = 0;

    // set PD coefficients
    Ki_throttle = 0.2;
    Ki_elevator = 0.0001;
    Kp_throttle = 0.5;
    Kp_elevator = 0.05;
}

double PD_controller::elevator_cmd(double err_altitude){
    integral_err_altitude += err_altitude;
    return Kp_elevator * err_altitude + Ki_elevator * integral_err_altitude;
}

double PD_controller::throttle_cmd(double err_velocity){
    integral_err_velocity += err_velocity;
    return Kp_throttle * err_velocity + Ki_throttle * integral_err_velocity;
}