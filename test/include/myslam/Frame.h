//
// Created by rookie-lu on 23-9-25.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include <vector>

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include "myslam/Feature.h"
#include "myslam/MapPoint.h"

namespace myslam
{
    class Frame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Frame> Ptr;

        Frame() = default;

        Frame(cv::Mat &&leftIMG, cv::Mat &&rightIMG)
                : m_leftIMG(leftIMG), m_rightIMG(rightIMG)
        {}

        static Ptr createFrame(cv::Mat &&leftIMG, cv::Mat &&rightIMG)
        {
            static std::size_t frameNum = 0;
            Ptr frame = std::make_shared<Frame>(std::move(leftIMG), std::move(rightIMG));
            frame->m_frameID = frameNum++;
            return frame;
        }

        void initFrame();

        void linkMapPoint(const std::vector<MapPoint::Ptr> &mapPoints);

        std::vector<Feature::Ptr> &getLeftFeatures()
        {
            return m_leftFeatures;
        }

        std::vector<Feature::Ptr> &getRightFeatures()
        {
            return m_rightFeatures;
        }

        const Sophus::SE3d &pose() const
        { return m_pose; }

        Sophus::SE3d &pose()
        { return m_pose; }

        const std::size_t &getID() const
        {
            assert(m_isKeyframe == true);
            return m_keyframeID;
        }

        const std::size_t &getFrameID() const
        { return m_frameID; }

        const cv::Mat &getLeftIMG() const
        { return m_leftIMG; }

        cv::Mat &getLeftIMG()
        { return m_leftIMG; }


    private:
        bool m_isKeyframe = false;
        std::size_t m_frameID = -1;
        std::size_t m_keyframeID = -1;

        Sophus::SE3d m_pose;  // Tcw
        std::vector<Feature::Ptr> m_leftFeatures;
        std::vector<Feature::Ptr> m_rightFeatures;
        cv::Mat m_leftIMG, m_rightIMG;
        static cv::Ptr<cv::GFTTDetector> m_gtffDetector;
    };

} // myslam

#endif //MYSLAM_FRAME_H
