//
// Created by rookie-lu on 23-9-25.
//

#include "myslam/Frame.h"

namespace myslam
{
    cv::Ptr<cv::GFTTDetector> Frame::m_gtffDetector = cv::GFTTDetector::create(150); // todo: 将150写入config文件

    void Frame::initFrame()
    {
        m_leftFeatures.clear();
        m_rightFeatures.clear();

        static std::size_t keyNum = 0;
        m_isKeyframe = true;
        m_keyframeID = keyNum++;

        std::vector<cv::Point2f> leftPoints, rightPoints;
        std::vector<cv::KeyPoint> leftKeyPoints;
        std::vector<uchar> status;
        std::vector<float> err;

        m_gtffDetector->detect(m_leftIMG, leftKeyPoints);
        for (auto &kp: leftKeyPoints) leftPoints.push_back(kp.pt);
        cv::calcOpticalFlowPyrLK(m_leftIMG, m_rightIMG, leftPoints, rightPoints, status, err);

        for (std::size_t idx = 0; idx < status.size(); ++idx) {
            if (status[idx] == 0) continue;

            Feature::Ptr leftFeature = std::make_shared<Feature>(
                    Eigen::Vector2d(leftPoints[idx].x, leftPoints[idx].y));
            Feature::Ptr rightFeature = std::make_shared<Feature>(
                    Eigen::Vector2d(rightPoints[idx].x, rightPoints[idx].y));

            m_leftFeatures.emplace_back(std::move(leftFeature));
            m_rightFeatures.emplace_back(std::move(rightFeature));
        }
    }

    void Frame::linkMapPoint(const std::vector<MapPoint::Ptr> &mapPoints)
    {
        assert(mapPoints.size() == m_leftFeatures.size());
        for (int i = 0; i < mapPoints.size(); ++i) {
            const MapPoint::Ptr &mapPoint = mapPoints[i];
            const Feature::Ptr &feature = m_leftFeatures[i];
            mapPoint->setFeature(feature);
            feature->setMapPoint(mapPoint);
        }
    }
} // myslam