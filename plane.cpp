#include "plane.hpp"
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

Plane::Plane(double cg_distance, double wing_distance, double tail_distance, double aoa, double h_0 ,double V_0, double theta_init){
    set_env_cond();
    set_airfoil_cond();
    set_design_param(cg_distance, wing_distance, tail_distance);
    set_init_cond(aoa, h_0, V_0, theta_init);
    set_forces();
    set_matrices();
    set_forces();
}


double Plane::get_cL(){
    return cL_0 + alpha * cL_alpha;
}

double Plane::get_cD(){
    return cD_0 + alpha * cD_alpha;
}

void Plane::set_env_cond(){
    g   = 9.81;
    rho = 1.225; // air density [kg/m^3]
}

void Plane::set_airfoil_cond(){
    // for NACA2414 Re=200,000
    cL_0     = 0.232;
    cL_alpha = 6.606; 
    cD_0     = 0.01;
    cD_alpha = 0.94*alpha - 0.018;
    cL_alpha_tail = -6.606;
}

void Plane::set_init_cond(double aoa, double h_0 ,double V_0, double theta_init){
    double theta, q0 = 0;
    init_altitude   = h_0; // [m]
    init_V          = V_0; // initial cruising velocity [m/s]
    V               = V_0;
    alpha           = aoa * M_PI / 180.0;
    u0 = init_V * cos(alpha);
    w0 = init_V * sin(alpha);
    q0 = 0; // by definition of the equilibrium point
    theta = theta_init * M_PI / 180.0;

    states_init << u0, w0, 0, theta;
    delta_states << 0, 0, 0, 0;
}

void Plane::set_design_param(double cg_distance, double wing_distance, double tail_distance){
    double wing2cg, tail2cg;

    mass = 12.0;
    I_yy = 2.0; // assumption of moment of inertia around y
    
    S_wing = 1.00; // assumption of wing area
    S_tail = 0.15; // assumption of tail area
    c_bar  = 0.30; // average chord length [m]

    h_cg = cg_distance;
    h_w  = wing_distance;
    h_t  = tail_distance;

}

void Plane::set_forces(){
    Q = 0.5 * rho * V*V; // dyanmic pressure
    L = Q * S_tail * get_cL();
    D = Q * S_tail * get_cD();
    W = mass * g;
    T = T_W_ratio * W;
}

void Plane::set_matrices(){

    double cL_u = 0, cD_u = 0; // change in cL and cD wrt u
    double X_u, X_w, Z_u, Z_w, M_u, M_w_dot, M_w, M_q;
    double cm_u = 0; // change in pitch moment wrt u
    double cm_alpha; // change in pitch moment wrt aoa

    // assumed parameters
    double cm_alpha_dot = -1.1; // change in pitch moment wrt q. Original formulation includes eta and epsilon -> hard to calculate
    double cm_elevator = -0.5; // change in pitch wrt elevator deflection [rad]
    double X_throttle = 1; // change in X wrt change in throttle [rad]
    double Z_elevator = -0.002; // change in Z wrt change in elevator deflectlion [rad]
    double M_elevator; // change in Z wrt change in elevator deflectlion [rad]
    
    
    cm_alpha = (cL_alpha*(h_cg - h_w)*S_wing + cL_alpha_tail*(h_t - h_cg)*S_tail) / (c_bar*S_wing);


    X_u = -(cD_u + 2*cD_0)*Q*S_wing / (mass*u0);
    X_w = -(cD_alpha - cL_0)*Q*S_wing / (mass*u0);
    Z_u = -(cL_u + 2*cL_0)*Q*S_wing / (mass*u0);
    Z_w = -(cL_alpha + cD_0)*Q*S_wing / (mass*u0);
    M_u = cm_u*(Q*S_wing*c_bar) / (I_yy*u0);
    M_w = cm_alpha*(Q*S_wing*c_bar) / (I_yy*u0);
    M_w_dot = cm_alpha_dot * (c_bar/(2*u0)) * ((Q*S_wing*c_bar)/(I_yy*u0));
    M_q = cm_alpha * Q / (I_yy*u0);

    M_elevator = cm_elevator*(Q*S_wing*c_bar) / (I_yy*u0);

    A << X_u, X_w, 0, -g,
         Z_u, Z_w, u0, 0,
         M_u+M_w_dot*Z_u, M_w+M_w_dot*Z_w, M_q+M_w_dot*u0, 0,
         0, 0, 1, 0;
    
    B << 2, X_throttle,
         Z_elevator, X_throttle,
         M_w_dot*Z_elevator + M_elevator, 0,
         0, 0;

}

Eigen::Matrix<double, 4, 1> Plane::get_states(){
    return states_init;
}

Eigen::Matrix<double, 4, 4> Plane::get_matrix_A(){
    return A;
}

Eigen::Matrix<double, 4, 2> Plane::get_matrix_B(){
    return B;
}


Eigen::Matrix<double, 4, 1> Plane::step(double delta_elevator, double delta_thrust, double dt){
    Eigen::Matrix<double, 4, 1> delta_states_derivative;
    Eigen::Matrix<double, 2, 1> control;
    //delta_states << delta_u, delta_w, delta_q ,delta_theta;
    control << delta_elevator, delta_thrust;
    delta_states_derivative = A*delta_states + B*control;
    //std::cout<<B<<std::endl;
    delta_states = delta_states + delta_states_derivative *dt;
    return delta_states;
}