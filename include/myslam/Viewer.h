//
// Created by rookie-lu on 23-9-27.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include "myslam/MapPoint.h"
#include "myslam/Frame.h"

namespace myslam
{
    class Viewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Viewer> Ptr;
        typedef std::vector<MapPoint::Ptr> PointsType;

        Viewer(const Viewer &other) = delete;

        Viewer &operator=(const Viewer &other) = delete;

        static Ptr getInstance();

        void setFrame(Frame::Ptr frame)
        { m_frame = std::move(frame); }

//        void setPoints(PointsType points)
//        { m_points = std::move(points); }

        void run();

        void drawIMG(cv::Mat &img);

        void drawFrame(pangolin::OpenGlRenderState& vis_camera);

        static void drawPoint(const Eigen::Vector3d &mapPoint);

    private:
        Viewer() = default;

        Frame::Ptr m_frame;
//        PointsType m_points;
    };

} // myslam

#endif //MYSLAM_VIEWER_H
