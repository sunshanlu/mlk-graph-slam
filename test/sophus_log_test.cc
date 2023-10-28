// 测试 Sophus::SE3d::log()和Eigen::Matrix::lpNorm()
#include "common_include.h"

using namespace myslam;
int main() {
    SE3d pose(SO3d(), Vec3d(3, 4, 0));
    Vec6d poseVec = pose.log();
    std::cout << poseVec.transpose() << std::endl;
    std::cout << poseVec.lpNorm<2>() << std::endl;
    return 0;
}