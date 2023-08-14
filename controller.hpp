#include <eigen3/Eigen/Dense>

class LQR_Controller{

    Eigen::Matrix4d A, Q;

    Eigen:: Matrix<double, 4, 2> B;

    Eigen::Matrix2d R;
    
    Eigen:: Matrix<double, 2, 4> K;

public:

    LQR_Controller(Eigen::Matrix4d A, Eigen::Matrix4d Q, Eigen:: Matrix<double, 4, 2> B, Eigen::Matrix2d R);
    
    Eigen:: Matrix<double, 2, 4> get_K();
    

};