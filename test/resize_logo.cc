#include <opencv2/opencv.hpp>

int main() {
    auto src = cv::imread("/home/rookie-lu/Pictures/MLK-SLAM.png", cv::IMREAD_COLOR);
    cv::Mat dst;
    cv::resize(src, dst, cv::Size(50, 50));
    cv::imwrite("/home/rookie-lu/Pictures/logo.png", dst);
    return 0;
}