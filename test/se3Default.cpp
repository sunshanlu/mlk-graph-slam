#include <iostream>

#include <sophus/se3.hpp>


int main()
{
    Sophus::SE3d pose;
    std::cout << pose.so3().matrix() << std::endl;
    std::cout << pose.translation() << std::endl;
}