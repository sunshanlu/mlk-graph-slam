//
// Created by rookie-lu on 23-9-27.
//

#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat outImg;
    cv::Mat img = cv::imread("/media/rookie-lu/DATA1/Dataset/KITTI-ODOM/00/image_0/000000.png", cv::IMREAD_GRAYSCALE);
    cv::cvtColor(img, outImg, CV_GRAY2BGR);
    cv::circle(outImg, cv::Point(20, 20), 10, cv::Scalar(255, 0, 0), -1);
    cv::imshow("outImg", outImg);
    cv::imshow("img", img);
    cv::waitKey(0);
    cv::destroyAllWindows();
}