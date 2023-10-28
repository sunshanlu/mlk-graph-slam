#include <sophus/se3.hpp>
#include <iostream>

int main(){
    Sophus::SO3d so3;
    std::cout << so3.matrix() << std::endl;
}