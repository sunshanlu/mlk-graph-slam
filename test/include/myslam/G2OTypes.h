//
// Created by rookie-lu on 23-9-26.
//

#ifndef MYSLAM_G2OTYPES_H
#define MYSLAM_G2OTYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <utility>

namespace myslam
{
    // 相机位姿顶点
    class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
    {
    public:
        typedef Eigen::Matrix<double, 6, 1> Vector6d;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        void setToOriginImpl() override
        { _estimate = Sophus::SE3d(); }

        void oplusImpl(const number_t *v) override
        {
            Vector6d update;
            update << v[0], v[1], v[2], v[3], v[4], v[5];
            _estimate = Sophus::SE3d::exp(update) * _estimate; // 左乘扰动
        }

        bool read(std::istream &is) override
        { return true; }

        bool write(std::ostream &os) const override
        { return true; }
    };

    // 路标顶点
    class VertexXYZ : public g2o::BaseVertex<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        void setToOriginImpl() override
        { _estimate = Eigen::Vector3d::Zero(); }

        void oplusImpl(const number_t *v) override
        {
            Eigen::Vector3d update;
            update << v[0], v[1], v[2];
            _estimate += update;
        }

        bool read(std::istream &is) override
        { return true; }

        bool write(std::ostream &os) const override
        { return true; }
    };

    // 仅估计位姿一元边
    class EdgePoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgePoseOnly(Eigen::Vector3d mapPoint, Eigen::Matrix3d K)
                : m_mapPoint(std::move(mapPoint)), m_K(std::move(K))
        {}

        void computeError() override;


//        void linearizeOplus() override;


        bool read(std::istream &is) override
        { return true; }

        bool write(std::ostream &os) const override
        { return true; }

    private:
        Eigen::Vector3d m_mapPoint;
        Eigen::Matrix3d m_K;
    };

    // 相机位姿和路标点二元边
    class EdgePosePoint : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexXYZ>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit EdgePosePoint(Eigen::Matrix3d K)
                : m_K(std::move(K))
        {}

        void computeError() override;

        bool read(std::istream &is) override
        { return true; }

        bool write(std::ostream &os) const override
        { return true; }

        void linearizeOplus() override;

    private:
        Eigen::Matrix3d m_K;
    };

} // myslam

#endif //MYSLAM_G2OTYPES_H
