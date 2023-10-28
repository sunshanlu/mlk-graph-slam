/**
 * @file        camera.h
 * @brief       定义Camera相机类
 * @author      rookie-lu (ssl2001@126.com)
 * @version     1.0
 * @date        Oct-28-2023
 *
 * @copyright   Copyright (c) 2023 rookie-lu
 *
 */

#pragma once

#include "common_include.h"
#include "config.h"

NAMESPACE_BEGIN

/**
 * @brief           Camera为双目相机类
 * @details         维护初始状态相机与世界坐标系的位姿和相机内参K
 */
class Camera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Camera> Ptr;

    Camera(const double &baseline, const Mat3d &K)
        : m_baseline(baseline)
        , m_K(K) {
        Vec3d t(-baseline, 0, 0);
        m_poses.push_back(SE3d());
        m_poses.push_back(SE3d(SO3d(), t));
    }

public:
    double m_baseline;         ///< 初始状态相机与世界坐标系的位姿Tcw(0)
    Mat3d m_K;                 ///< 相机的内参矩阵K
    std::vector<SE3d> m_poses; ///< Tc1w Tc2w
};

NAMESPACE_END