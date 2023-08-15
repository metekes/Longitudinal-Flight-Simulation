class PD_controller{
private:
    double integral_err_altitude, integral_err_velocity;
    double Ki_throttle, Ki_elevator;
    double Kp_throttle, Kp_elevator;

public:
    double elevator_cmd(double err_elevator);
    double throttle_cmd(double err_throttle);

    PD_controller();
};