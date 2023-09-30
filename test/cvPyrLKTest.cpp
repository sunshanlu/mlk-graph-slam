//
// Created by rookie-lu on 23-9-28.
//

#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat img1 = cv::imread("/media/rookie-lu/DATA1/Dataset/KITTI-ODOM/00/image_0/000000.png", cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread("/media/rookie-lu/DATA1/Dataset/KITTI-ODOM/00/image_0/000001.png", cv::IMREAD_GRAYSCALE);
    cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create(200);
    std::vector<cv::KeyPoint> keyPoints;
    std::vector<cv::Point2f> points1, points2;

    gftt->detect(img1, keyPoints);
    for (const auto &keyPoint: keyPoints) points1.push_back(keyPoint.pt);

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err);

    return 0;
}