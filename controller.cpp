#include"controller.hpp"
#include <eigen3/Eigen/Dense>

LQR_Controller::LQR_Controller(Eigen::Matrix4d _A, Eigen::Matrix4d _Q, Eigen:: Matrix<double, 4, 2> _B, Eigen::Matrix2d _R){
    A = _A;
    B = _B;
    Q = _Q;
    R = _R;
}

Eigen:: Matrix<double, 2, 4> LQR_Controller::get_K(){
    
    Eigen::Matrix4d P;
    Eigen:: Matrix<double, 2, 4> K;

    P  << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    for (int i = 0; i < 100; ++i) {
        K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        P = Q + A.transpose() * P * (A - B * K);
    }

    return K;

}