//
// Created by rookie-lu on 23-9-25.
//

#include <fstream>
#include <opencv2/opencv.hpp>
#include <fmt/format.h>

#include "myslam/Dataset.h"

namespace myslam
{
    Frame::Ptr Dataset::nextFrame()
    {
        static std::size_t imgNum = 0;
        std::string leftPath = fmt::format("{}/image_0/{:06}.png", m_prefix, imgNum);
        std::string rightPath = fmt::format("{}/image_1/{:06}.png", m_prefix, imgNum++);

        cv::Mat leftIMG = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
        cv::Mat rightIMG = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);

        if (leftIMG.data == nullptr || rightIMG.data == nullptr)
            return nullptr;
        cv::Mat leftIMGResized, rightIMGResized;
        cv::resize(leftIMG, leftIMGResized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);
        cv::resize(rightIMG, rightIMGResized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);

        return Frame::createFrame(std::move(leftIMGResized), std::move(rightIMGResized));
    }

    Camera::Ptr Dataset::createCamera()
    {
        std::string cameraPath = fmt::format("{}/calib.txt", m_prefix);
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
        t = K.inverse() * t;
        K = 0.5 * K;
        double baseline = t.norm();

//        std::cout << K << std::endl;
//        std::cout << t.transpose() << std::endl;

        Camera::Ptr camera = std::make_shared<Camera>(K, baseline);
        return std::move(camera);
    }
} // myslam