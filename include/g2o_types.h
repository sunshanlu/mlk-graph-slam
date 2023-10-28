#pragma once

#include "common_include.h"
#include "config.h"

NAMESPACE_BEGIN
/**
 * @brief   位姿顶点
 *
 */
class PoseVertex : public g2o::BaseVertex<6, SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override { _estimate = SE3d(); }

    bool read(std::istream &in) override { return false; }

    bool write(std::ostream &os) const override { return false; }

    void oplusImpl(const double *arr) override {
        Vec6d update;
        update << arr[0], arr[1], arr[2], arr[3], arr[4], arr[5];
        _estimate = SE3d::exp(update) * _estimate;
    }
};

/**
 * @brief   地图点顶点
 *
 */
class PointVertex : public g2o::BaseVertex<3, Vec3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override { _estimate = Vec3d(); }

    bool read(std::istream &in) override { return false; }

    bool write(std::ostream &os) const override { return false; }

    void oplusImpl(const double *arr) override {
        Vec3d update;
        update << arr[0], arr[1], arr[2];
        _estimate = _estimate + update;
    }
};

/**
 * @brief   只含有位姿的一元边
 *
 */
class PoseEdge : public g2o::BaseUnaryEdge<2, Vec2d, PoseVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool read(std::istream &in) override { return false; }

    bool write(std::ostream &out) const override { return false; }

    PoseEdge(const Vec3d &worldPoint, const Mat3d &K)
        : m_point(worldPoint)
        , m_K(K) {}

    void computeError() override;

    void linearizeOplus() override;

private:
    Vec3d m_point; ///< 地图点的位置信息
    Mat3d m_K;     ///< 相机的内参矩阵K
};

/**
 * @brief   位姿和地图点二元边
 *
 */
class PosePointEdge : public g2o::BaseBinaryEdge<2, Vec2d, PoseVertex, PointVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool read(std::istream &in) override { return false; }

    bool write(std::ostream &out) const override { return false; }

    void computeError() override;

    PosePointEdge(const Mat3d &K)
        : m_K(K) {}

    virtual void linearizeOplus() override;

private:
    Mat3d m_K; ///< 相机的内参矩阵K
};

NAMESPACE_END