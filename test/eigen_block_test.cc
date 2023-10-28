#include <Eigen/Dense>
#include <iostream>

double data[12] = {7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, -3.861448000000e+02,
                   0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00,
                   0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00};

int main() {
    Eigen::Matrix3d K;
    Eigen::Vector3d t;
    K << data[0], data[1], data[2], data[4], data[5], data[6], data[8], data[9], data[10];
    t << data[3], data[7], data[11];

    std::cout << K << std::endl;
    std::cout << t.transpose() << std::endl;

    t = K.inverse() * t;
    K.block<2, 3>(0, 0) = 0.5 * K.block<2, 3>(0, 0);
    std::cout << K << std::endl;
    std::cout << t.transpose() << std::endl;

    std::cout << t.head<2>() << std::endl;

    return 0;
}
