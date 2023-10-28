/**
 * @file        viewer.h
 * @brief       展示Frontend的m_currFrame和Map的m_activeFrames和activePoints
 * @author      rookie-lu (ssl2001@126.com)
 * @version     1.0
 * @date        Oct-16-2023
 *
 * @copyright   Copyright (c) 2023 rookie-lu
 *
 */
#pragma once

#include "camera.h"
#include "common_include.h"
#include "config.h"
#include "map.h"

NAMESPACE_BEGIN

/**
 * @brief   绘制激活的地图信息和前端的当前帧信息
 * @details 当前帧绘制：前端在完成当前帧的位姿估计后进行（这是通过前端执行顺序保证的）
 *          绘制激活地图点：后端优化完成并完成异常地图点剔除后进行（这是通过后端执行顺序保证的）
 *          绘制激活关键帧：没有顺序要求
 */
class Viewer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Viewer> Ptr;
    typedef std::vector<MapPoint::Ptr> PointsType;

    Viewer(const Viewer &) = delete;
    Viewer &operator=(const Viewer &) = delete;
    static Ptr getInstance() {
        static Ptr viewer(new Viewer);
        return viewer;
    }

    void run();

    void drawFrame(const Frame &frame, const float *color);

    void drawCurrFrame() {
        std::unique_lock<std::mutex> lck(m_mutex);
        drawFrame(*m_frame, Config::CURR_FRAME_COLOR);
    }

    void drawImage(pangolin::GlTexture &texture);

    void drawKeyFrames();

    void updateFrame(Frame::Ptr frame) {
        std::unique_lock<std::mutex> viewLck(m_mutex);
        m_frame = frame;
    }

    void setMap(const Map::Ptr &map) { m_map = map; }

    void setCamera(const Camera::Ptr &camera) { m_camera = camera; }

    void drawMapPoints();

    void updatePoints(PointsType &&points) {
        std::unique_lock<std::mutex> lck(m_mutex);
        m_points = std::move(points);
    }

    void followCurrFrame(pangolin::OpenGlRenderState &renderState);

    bool getFlag() {
        std::unique_lock<std::mutex> lck(m_mutex);
        return m_isStop;
    }

    void setFlag() {
        std::unique_lock<std::mutex> lck(m_mutex);
        m_isStop = true;
    }

private:
    Viewer() = default;
    Map::Ptr m_map;        ///< 地图，用来提供激活地图点和激活关键帧
    Camera::Ptr m_camera;  ///< 相机，用来提供相机位姿
    std::mutex m_mutex;    ///< Viewer的互斥量维护m_frame的线程安全
    Frame::Ptr m_frame;    ///< 前端刚刚完成位姿估计的那一帧
    PointsType m_points;   ///< 需要绘制的地图点信息
    bool m_isStop = false; ///< 展示线程结束标识符
};

NAMESPACE_END