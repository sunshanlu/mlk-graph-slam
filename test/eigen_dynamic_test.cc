#include "common_include.h"
#include <iostream>

using namespace myslam;

int main() {
    MatXd matx4d(4, 4);
    Eigen::Matrix<double, 1, 4> vec4d;
    for (int i = 0; i < 4; ++i) {
        for (int j = i; j < i + 4; ++j)
            vec4d << j, j + 1, j + 2, j + 3;
        matx4d.block<1, 4>(i, 0) = vec4d;
    }
    std::cout << matx4d << std::endl;

    return 0;
}