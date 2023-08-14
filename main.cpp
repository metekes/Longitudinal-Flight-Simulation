#include"plane.hpp"
#include <iostream>
#include <fstream>
#include <math.h>
#include <iomanip>

int main(){

double dt = 0.01, dt_record = 0.2; // dt: step size [s], dt_record: data recording period [s]
double altitude = 200,V_init = 17 ; // initial altitude [m] and velocity [m/s]
double AoA_init = 5, theta_init = 3; // initial AoA [deg] and theta [deg]

double h_cg = 0.3, h_w = 0.4, h_t = 0.9; // h_cg: distance from nose to cg
                                         // h_w: distance from nose to wing
                                         // h_t: distance from nose to tail

// initialize control commands
double delta_elevator_cmd = 0.1, delta_thrust_cmd = 0.2;

// initialize the variables to be stored
double t, V, V_ref, altitude_ref, theta, alpha, u, w;

int count=0; // variable to keep track of data recording frequency

// initialize states and delta_states
Eigen::Matrix<double, 4, 1> delta_states;
Eigen::Matrix<double, 4, 1> states;

// initialize plane
Plane plane(h_cg, h_w, h_t, AoA_init, altitude, V_init, theta_init);

// get initial states
states = plane.get_states();

// initialize file to store data
std::fstream f;
f.open("data.txt", std::ios::out);

// set number of digits for the decimal
f << std::fixed << std::setprecision(3);

// set data for t=0
V = V_init;
theta = theta_init;
alpha = AoA_init;

// start solving
for(double i = 0; i<=10000; i = i+dt){

    // save the data [5Hz]
    if(count%int(dt_record/dt) == 0){
    f << i << ", ";
    f << V << ", ";
    f << V_ref << ", ";
    f << altitude << ", ";
    f << altitude_ref << ", ";
    f << theta << ", ";
    f << alpha  << "\n";
    }

    // run the model for dt
    delta_states = plane.step(delta_elevator_cmd, delta_thrust_cmd, dt);

    // update the states
    u = states(0) + delta_states(0);
    w = states(1) + delta_states(1);
    theta = states(3) + delta_states(3);
    theta = theta * 180.0/M_PI;

    V = sqrt(u*u + w*w);
    
    alpha = atan(w/u) * 180.0/M_PI;

    altitude = altitude + (u*sin(theta*M_PI/180.0)-w*cos(theta*M_PI/180.0)) * dt;

    count++;
}

f.close();

}
