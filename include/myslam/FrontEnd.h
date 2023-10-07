//
// Created by rookie-lu on 23-9-25.
//

#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <mutex>

#include <g2o/core/sparse_optimizer.h>

#include "myslam/Feature.h"
#include "myslam/Frame.h"
#include "myslam/MapPoint.h"
#include "myslam/Map.h"
#include "myslam/Dataset.h"
#include "myslam/Camera.h"
#include "myslam/G2OTypes.h"
#include "myslam/Viewer.h"

namespace myslam
{

    class FrontEnd
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<FrontEnd> Ptr;

        FrontEnd(const FrontEnd &other) = delete;

        FrontEnd &operator=(const FrontEnd &other) = delete;

        static Ptr getInstance();

        void setDataset(Dataset::Ptr dataset)
        { m_dataset = std::move(dataset); }

        void setMap(Map::Ptr map)
        { m_map = std::move(map); }

        void setCamera(Camera::Ptr camera)
        { m_camera = std::move(camera); }

        void setViewer(Viewer::Ptr viewer)
        { m_viewer = std::move(viewer); }

        void run();

        void track();  // 特征点追踪

        std::vector<MapPoint::Ptr> initKeyFrame(Frame::Ptr &frame);  // 初始化关键帧，进行三角测量

        Eigen::Vector3d triangulate(const Eigen::Vector2d &refPoint, const Eigen::Vector2d &currPoint);

        // todo：进行位姿估计
        bool poseEstimate(Sophus::SE3d &firstOptiRet);

        int outlierDeter(VertexPose *vertexPose, g2o::SparseOptimizer &graph, std::vector<EdgePoseOnly *> &edges,
                         std::vector<Feature::Ptr> &features, Sophus::SE3d &firstOptiRet);

    private:
        FrontEnd() = default;

        Dataset::Ptr m_dataset;
        Map::Ptr m_map;
        Camera::Ptr m_camera;
        Frame::Ptr m_refFrame;
        Frame::Ptr m_currFrame;
        Viewer::Ptr m_viewer;
    };

} // myslam

#endif //MYSLAM_FRONTEND_H
