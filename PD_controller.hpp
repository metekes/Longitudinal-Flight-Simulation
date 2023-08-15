class PD_controller{
private:
    double integral_err_altitude, integral_err_velocity; // integral errors

    double Ki_throttle, Ki_elevator; // integral coefficients
    double Kp_throttle, Kp_elevator; // proportional coefficients

public:
    // PUBLIC METHODS //
    double elevator_cmd(double err_elevator);

    double throttle_cmd(double err_throttle);

    PD_controller(); // constructor
};