/**
 * @file        backend.h
 * @brief       编写后端优化类Backend
 * @author      rookie-lu (ssl2001@126.com)
 * @version     1.0
 * @date        Oct-20-2023
 *
 * @copyright   Copyright (c) 2023 rookie-lu
 *
 */

#pragma once
#include "camera.h"
#include "common_include.h"
#include "g2o_types.h"
#include "map.h"
#include "viewer.h"
#include "config.h"

NAMESPACE_BEGIN

/**
 * @brief   后端优化类
 *
 */
class Backend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Backend> Ptr;

    static Ptr getInstance() {
        static Ptr backend(new Backend);
        return backend;
    }

    void getPointsAndFrames(std::vector<Frame::Ptr> &frames, std::vector<MapPoint::Ptr> &points);

    void addVertexAndEdge(std::unordered_map<std::size_t, PoseVertex *> &poseVertexes,
                          std::unordered_map<std::size_t, PointVertex *> &pointVertexes,
                          std::unordered_map<Feature::Ptr, PosePointEdge *> &edges,
                          std::vector<Frame::Ptr> &frames, std::vector<MapPoint::Ptr> &points,
                          g2o::SparseOptimizer &graph);

    void run();

    void setMap(const Map::Ptr &map) { m_map = map; }

    void setCamera(const Camera::Ptr &camera) { m_camera = camera; }

    void setViewer(const Viewer::Ptr &viewer) { m_viewer = viewer; }

    void optimize(std::unique_lock<std::mutex> &lck);

    void setOptValue(std::unordered_map<std::size_t, PoseVertex *> poseVertexes,
                     std::unordered_map<std::size_t, PointVertex *> pointVertexes);

    double detectOutlier(g2o::SparseOptimizer &graph,
                         std::unordered_map<Feature::Ptr, PosePointEdge *> &edges);

    Backend(const Backend &other) = delete;

    Backend &operator=(const Backend &other) = delete;

private:
    Backend() = default;
    Map::Ptr m_map;        ///< 地图
    Camera::Ptr m_camera;  ///< 相机，保存内参
    Viewer::Ptr m_viewer;  ///< 全局展示对象
    bool m_isStop = false; ///< 后端停止标识
};

NAMESPACE_END