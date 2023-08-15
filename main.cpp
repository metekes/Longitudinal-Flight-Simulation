#include "plane.hpp"
#include <iostream>
#include <fstream>
#include <math.h>
#include <iomanip>
#include "PD_controller.hpp"


int main(){

double dt = 0.01, dt_record = 0.2, t_max = 300; // dt: step size [s], dt_record: data recording period [s], t_max: total time to compute the states [s]
double altitude = 200, V_init = 17 ; // initial altitude [m] and velocity [m/s]
double AoA_init = 5, theta_init = 3; // initial AoA [deg] and theta [deg]

double h_cg = 0.3, h_w = 0.5, h_t = 0.9; // h_cg: distance from nose to cg
                                         // h_w: distance from nose to wing
                                         // h_t: distance from nose to tail

// initialize control commands
double delta_elevator_cmd = 0, delta_throttle_cmd = 0;

// initilize errors
double err_velocity = 0, err_altitude = 0;

// initialize the variables to be stored
double t, V, V_ref, altitude_ref, theta, alpha, u, w;

int count=0; // variable to keep track of data recording frequency

// get control inputs
std::cout << "Reference Velocity [m/s]: ";
std::cin >> V_ref;
std::cout << "Reference Attitude [m]: ";
std::cin >> altitude_ref;

// initialize states and delta_states
Eigen::Matrix<double, 4, 1> delta_states;
Eigen::Matrix<double, 4, 1> states;

// initialize forces
Eigen::Matrix<double, 4, 1> forces;

// initialize plane
Plane plane(h_cg, h_w, h_t, AoA_init, altitude, V_init, theta_init);

// initilize PD controller
PD_controller pd_controller;

// get initial states
states = plane.get_states();

// initialize file to store data
std::fstream f;
f.open("data.txt", std::ios::out);
f << std::fixed << std::setprecision(3); // set number of digits for the decimal

// set data for t=0
V = V_init;
theta = theta_init;
alpha = AoA_init;


// start solving
for(double i = 0; i<=t_max; i = i+dt){

    // save the data [5Hz]
    if(count%int(dt_record/dt) == 0){
    f << i << ", ";
    f << V << ", ";
    f << V_ref << ", ";
    f << altitude << ", ";
    f << altitude_ref << ", ";
    f << theta << ", ";
    f << alpha << "\n";

    /// IF YOU WANT TO KEEP TRACK OF THE FORCES IN THE TERMINAL, UNCOMMENT THE FOLLOWING LINES ///
    /*
    forces = plane.get_forces(V, alpha*M_PI/180.0);
    std::cout << "time = " << i << std::endl;
    std::cout << "Lift: " << forces(0) << std::endl;
    std::cout << "Drag: " << forces(1) << std::endl;
    std::cout << "Thrust: " << forces(2) << std::endl;
    std::cout << "Weight: " << forces(3) << "\n" << std::endl;
    */
    }

    // run the model for dt
    delta_states = plane.step(delta_elevator_cmd, delta_throttle_cmd, dt);

    // update the states
    u = states(0) + delta_states(0);
    w = states(1) + delta_states(1);
    theta = states(3) + delta_states(3);
    theta = theta * 180.0/M_PI; // rad2deg

    V = sqrt(u*u + w*w);
    
    alpha = atan(w/u) * 180.0/M_PI;  // rad2deg
    altitude = altitude + (u*sin(theta*M_PI/180.0)-w*cos(theta*M_PI/180.0)) * dt;

    // compute errors
    err_altitude = (altitude_ref - altitude);
    err_velocity = (V_ref - V);

    // apply control inputs
    // For the sake of simplicty, I assumed the altitude is mostly affected by elevator_cmd and the velocity is mostly affected by throttle_cmd
    delta_elevator_cmd = pd_controller.elevator_cmd(err_altitude);
    delta_throttle_cmd   = pd_controller.throttle_cmd(err_velocity);

    count++;
}

f.close(); // close data.txt file
return 0;
}
