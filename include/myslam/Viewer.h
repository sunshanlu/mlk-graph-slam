//
// Created by rookie-lu on 23-9-27.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <memory>
#include <vector>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include "myslam/MapPoint.h"
#include "myslam/Frame.h"
#include "myslam/Map.h"

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

        void setMap(Map::Ptr &map)
        { m_map = map; }

        static Ptr getInstance();

        void setFrame(Frame::Ptr frame)
        {
            std::unique_lock<std::mutex> lck(m_mutex);
            m_frame = std::move(frame);
        }

        void run();

        void drawIMG(cv::Mat &img);

        // void drawFrame(pangolin::OpenGlRenderState &vis_camera);

        Eigen::Matrix4d drawFrame(Frame::Ptr &frame, bool isKf = false) const;

        void drawPoint(const Eigen::Vector3d &mapPoint);

        void drawCurrFrame(pangolin::OpenGlRenderState &vis_camera)
        {
            std::unique_lock<std::mutex> lck(m_mutex);
            cv::Mat img;
            drawIMG(img);
            cv::imshow("currLeftIMG", img);
            Eigen::Matrix4d poseMatrix = drawFrame(m_frame);
            pangolin::OpenGlMatrix matrix(poseMatrix);
            vis_camera.Follow(matrix, true);
        }

        void drawActiveMap();

    private:
        Viewer() = default;

        Frame::Ptr m_frame;

        std::mutex m_mutex;

        Map::Ptr m_map;

        const float m_sz = 1.0;
        const float m_line_width = 2.0;
        const float m_fx = 400;
        const float m_fy = 400;
        const float m_cx = 512;
        const float m_cy = 384;
        const float m_width = 1080;
        const float m_height = 768;
        const float m_frameColor[3] = {1.0f, 0.0f, 0.0f};  // todo: 写入config文件
        const float m_kfColor[3] = {0.0f, 0.0f, 1.0f};
        const float m_pointColor[3] = {1.0f, 0.0f, 0.0f};
    };
} // myslam

#endif //MYSLAM_VIEWER_H
