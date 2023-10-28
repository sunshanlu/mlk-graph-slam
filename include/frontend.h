/**
 * @file        frontend.h
 * @brief       定义Frontend前端类
 * @author      rookie-lu (ssl2001@126.com)
 * @version     1.0
 * @date        Oct-28-2023
 *
 * @copyright   Copyright (c) 2023 rookie-lu
 *
 */
#pragma once
#include "camera.h"
#include "common_include.h"
#include "config.h"
#include "dataset.h"
#include "g2o_types.h"
#include "map.h"
#include "viewer.h"

NAMESPACE_BEGIN

/**
 * @brief           TrackStatus来表示追踪和位姿估计后的状态
 * INIT_TRACKING    代表前端处在关键帧初始化状态
 * TRACK_GOOD       代表前端运转良好（可用MapPoint数目大于50）
 * TRACK_BAD        代表前端运转较差（可用MapPoint数目在20-50）
 * TRACK_LOST       代表前端追踪失败，需要关键帧初始化（可用MapPoint数目小于20）
 * TRACK_ERROR      当关键帧初始化后，依然TRACK_LOST，前端退出，程序结束
 */
enum class TrackStatus { INIT_TRACKING, TRACK_GOOD, TRACK_BAD, TRACK_LOST, TRACK_ERROR };
void printPose(const SE3d &pose);

class Frontend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Frontend> Ptr;
    Frontend(const Frontend &) = delete;
    Frontend &operator=(const Frontend &) = delete;

    static Ptr getInstance() {
        static Ptr frontend(new Frontend);
        return frontend;
    }

    void run();

    void addFrame(const Frame::Ptr &frame);

    bool triangulate(const std::vector<SE3d> &poses, const std::vector<Vec2d> &pixelPoints,
                     Vec3d &worldPoint);

    int triNewPoint(Frame::Ptr &frame, std::vector<MapPoint::Ptr> &newPoints);

    void setCamera(const Camera::Ptr &camera) { m_camera = camera; }

    void setMap(const Map::Ptr &map) { m_map = map; }

    void setDataset(const Dataset::Ptr &dataset) { m_dataset = dataset; }

    void setViewer(const Viewer::Ptr &viewer) { m_viewer = viewer; }

    int trackCurrFrame();

    void initTrack();

    int estimatePose();

    int detectOutlier(std::vector<Feature::Ptr> &features, std::vector<PoseEdge *> &edges,
                      g2o::SparseOptimizer &optimizer, PoseVertex *vertex);

    void track() {
        int trackNum = trackCurrFrame();
        SPDLOG_INFO("==================================================================");
        SPDLOG_INFO("参考帧: {:06}\t当前帧: {:06} 追踪成功的特征点数目为: {:03}", m_refFrame->m_id,
                    m_currFrame->m_id, trackNum);
        int inlinerNum = estimatePose();

        /// @note 维护位姿差m_diffPose
        m_diffPose = m_currFrame->pose() * m_refFrame->pose().inverse();
        SPDLOG_INFO("对当前帧: {:06} 进行位姿估计 非异常点数目为: {:03} 当前帧位姿为:",
                    m_currFrame->m_id, inlinerNum);
        printPose(m_currFrame->pose());
        if (inlinerNum < Config::MIN_MAPPOINTS_NUM_FOR_TRACK) {
            initTrack();
        }
        SPDLOG_INFO("==================================================================\n");
    }

private:
    Frontend() = default;
    Frame::Ptr m_refFrame;  /// 参考帧，左图提供待追踪的Feature
    Frame::Ptr m_currFrame; /// 当前帧，参考参考帧进行Feature追踪
    bool m_isStop = false;  /// 前端线程结束标识
    Camera::Ptr m_camera;   /// 相机参数
    Dataset::Ptr m_dataset; /// 数据集
    Map::Ptr m_map;         /// 地图
    SE3d m_diffPose;        /// 上次位姿估计的位姿差Tcr
    Viewer::Ptr m_viewer;   /// 可视化
    TrackStatus m_status = TrackStatus::INIT_TRACKING; /// 追踪状态
};

NAMESPACE_END