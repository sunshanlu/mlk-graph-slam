#include <fstream>
#include <fmt/format.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

int main()
{
    std::string cameraPath = fmt::format("{}/calib.txt", "/media/rookie-lu/DATA1/Dataset/KITTI-ODOM/00");
    std::ifstream ifs(cameraPath);
    std::string cameraName;
    Eigen::Matrix3d K;
    Eigen::Vector3d t;
    double data[12];

    for (std::size_t idx = 0; idx < 2; ++idx) {
        ifs >> cameraName;
        for (double &i: data)
            ifs >> i;
        if (idx == 0) {
            K << data[0], data[1], data[2], data[4], data[5], data[6], data[8], data[9], data[10];
        } else {
            t << data[3], data[7], data[11];
        }
    }
    std::cout << K << std::endl;
    std::cout << t.transpose() << std::endl;
    t = K.inverse() * t;

    return 0;
}