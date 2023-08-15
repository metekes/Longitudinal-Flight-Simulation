#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

class Plane{
    
private:
    ///// PRIVATE ATTRIBUTES ////

    // environmental conditions
    double g, rho;

    // for NACA2414
    double cL_0, cL_alpha, cD_0, cD_alpha;
    double cL_alpha_tail;

    // design parameters
    double mass, I_yy;
    double S_wing, S_tail, c_bar;
    double h_cg, h_w, h_t;

    // initial conditions
    double init_altitude, init_V;
    double init_aoa; // in degrees
    double u0, w0;

    // forces
    const double T_W_ratio = 1.1;
    double L, D, W, T;
    double Q; // dynamic pressure

    // states
    double delta_u, delta_w, delta_q, delta_theta;
    double V;
    double alpha;

    // state-space matirces
    Eigen::Matrix4d A;
    Eigen::Matrix<double,4,2> B;
    Eigen::Matrix<double, 4, 1> delta_states, states_init;

    
    ///// PRIVATE METHODS ////
    void set_env_cond();

    void set_airfoil_cond(double aoa);

    void set_forces(double V, double aoa);

    void set_matrices();

    double get_cL(double aoa);

    double get_cD(double aoa);

    void set_design_param(double h_cg, double h_w, double h_t);

    void set_init_cond(double aoa, double h_0 ,double V_0, double theta_0);

public:
    // constructor
    Plane(double h_cg, double h_w, double h_t, double aoa, double h_0 ,double V_0, double theta_init);

    ///// PUBLIC METHODS ////
    Eigen::Matrix<double, 4, 1> get_forces(double V, double alpha);
    Eigen::Matrix<double, 4, 1> get_states();

    Eigen::Matrix<double, 4, 1> step(double delta_elevator, double delta_thrust, double dt);

};