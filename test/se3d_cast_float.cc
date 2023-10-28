#include <Eigen/Dense>
#include <iostream>
#include <sophus/se3.hpp>

int main() {
    Sophus::SE3d pose(Sophus::SO3d(), Eigen::Vector3d(1, 2, 3));
    Eigen::Matrix4f matrix = pose.matrix().cast<float>();
    float *data = matrix.data();
    for (int i = 0; i < 16; ++i) {
        std::cout << *data++ << " ";
    }
    std::cout << std::endl;
    return 0;
}