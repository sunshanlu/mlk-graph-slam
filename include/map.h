/**
 * @file        map.h
 * @brief       定义slam系统的数据结构包括Feature、Frame、MapPoint、Map
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

NAMESPACE_BEGIN
struct MapPoint;

/**
 * @brief   Feature维护了帧的特征点信息
 * @note    Feature维护了其与MapPoint的关系，像素坐标
 *
 */
struct Feature {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Feature> Ptr;

    /**
     * @brief   Frame的简单工厂设计模式
     *
     * @return Ptr
     */
    static Ptr create() {
        Ptr feature(new Feature);
        return feature;
    }

    std::weak_ptr<MapPoint> m_mapPoint; /// 维护的MapPoint的weak_ptr
    Vec2d m_position;                   /// 维护了像素坐标系下的位置信息
};

/**
 * @brief       MapPoint为地图点
 * @note        MapPoint在三角测量后得到（采用SVD分解的方法）
 * @attention   MapPoint以第一帧的相机坐标系作为世界坐标系
 */
struct MapPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<MapPoint> Ptr;

    /**
     * @brief   MapPoint的简单工厂设计模式
     * @note    create()保证了MapPoint中m_id的唯一性
     *
     * @return Ptr
     */
    static Ptr create() {
        static std::size_t pointNum = 0;
        Ptr point(new MapPoint);
        point->m_id = pointNum++;
        return point;
    }

    bool m_isOutlier = false; /// outlier标识
    int m_featureNum = 0;     /// 维护由**关键帧**指向地图点的Feature
    std::size_t m_id;         /// 唯一的id信息
    Vec3d m_position;         /// 世界坐标系上位置信息
};

/**
 * @brief   Frame维护了一帧内的信息
 * @note    m_rightFeatures中的Feature只起到三角化的作用
 */
class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Frame> Ptr;

    /**
     * @brief   Frame只提供左右图的构造方式
     * @param   [in]leftIMG: 左图
     * @param   [in]rightIMG:右图
     *
     */
    Frame(cv::Mat leftIMG, cv::Mat rightIMG)
        : m_leftIMG(std::move(leftIMG))
        , m_rightIMG(std::move(rightIMG)) {
        static std::size_t frameNum = 0;
        m_id = frameNum++;
    }

    std::vector<Feature::Ptr> &getLeftFeatures() { return m_leftFeatures; }
    std::vector<Feature::Ptr> &getRightFeatures() { return m_rightFeatures; }

    int initKeyFrame(const Camera::Ptr &camera);

    static double distance(const Frame &left, const Frame &right);

    SE3d &pose() { return m_pose; }
    const SE3d &pose() const { return m_pose; }

private:
    std::vector<Feature::Ptr> m_leftFeatures;  /// 左图特征点
    std::vector<Feature::Ptr> m_rightFeatures; /// 右图特征点（不关联MapPoint）
    SE3d m_pose;                               /// Frame的位姿Tcw

public:
    std::size_t m_keyID = -1;  /// 关键帧id，如果fram是关键帧
    std::size_t m_id;          /// 索引（唯一）
    cv::Mat m_leftIMG;         /// 左图
    cv::Mat m_rightIMG;        /// 右图
    bool m_isKeyFrame = false; /// 关键帧标识
};

/**
 * @brief       Map用作前端、展示和后端的交互中心（单例设计模式）
 * @attention   使用m_mutex和m_cond来保证Map的线程安全
 */
class Map {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<std::size_t, Frame::Ptr> FramesType;
    typedef std::unordered_map<std::size_t, MapPoint::Ptr> PointsType;

    Map(const Map &other) = delete;
    Map &operator=(const Map &other) = delete;

    void insertMapPoints(const std::vector<MapPoint::Ptr> &mapPoints) {
        for (auto &point : mapPoints)
            m_points.insert(std::make_pair(point->m_id, point));
    }

    /**
     * @brief           添加关键帧到Map中
     * @details         完成m_frames、m_activeFrames和m_activePoints的初始化
     *
     * @param           [in ]keyFrame:待添加到Map中的关键帧
     *
     */
    void insertKeyFrame(const Frame::Ptr &keyFrame) {
        assert(keyFrame->m_isKeyFrame == true);
        m_frames.insert(std::make_pair(keyFrame->m_keyID, keyFrame));
        if (m_activeFrames.size() == 7)
            removeOldFrame(keyFrame);
        m_activeFrames.insert(std::make_pair(keyFrame->m_keyID, keyFrame));

        m_activePoints.clear();
        for (auto &frame : m_activeFrames) {
            auto &leftFeatures = frame.second->getLeftFeatures();
            for (auto &feature : leftFeatures) {
                auto mapPoint = feature->m_mapPoint.lock();
                if (mapPoint == nullptr)
                    continue;
                m_activePoints.insert(std::make_pair(mapPoint->m_id, mapPoint));
            }
        }
    }

    static Ptr getInstance() {
        static Ptr map(new Map);
        return map;
    }

    FramesType &getActiveFrames() { return m_activeFrames; }

    PointsType &getActivePoints() { return m_activePoints; }

    FramesType &getAllFrames() { return m_frames; }

    PointsType &getAllPoints() { return m_points; }

    void removeOldFrame(const Frame::Ptr &keyFrame);

    void cleanMap();

private:
    Map() = default;
    FramesType m_frames;       /// 存储slam过程中的关键帧
    PointsType m_points;       /// 存储slam过程中的地图点MapPoints
    FramesType m_activeFrames; /// 存储待后端优化的frames
    PointsType m_activePoints; /// 存储待后端优化的points

public:
    std::mutex m_mutex;             /// 地图锁，保证后端、前端和Viewer的线程安全
    std::condition_variable m_cond; /// 条件变量，保证后端优化效率
    bool m_isReady = false;         /// 搭配条件变量一起使 用
};

NAMESPACE_END