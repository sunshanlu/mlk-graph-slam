//
// Created by rookie-lu on 23-9-26.
//

#include "myslam/G2OTypes.h"

namespace myslam
{
//    void EdgePoseOnly::linearizeOplus()
//    {
//        const VertexPose *vertexPose = dynamic_cast<VertexPose *>(_vertices[0]);
//        const Sophus::SE3d &Tcw = vertexPose->estimate();
//        Eigen::Vector3d camPoint = Tcw * m_mapPoint;
//        double fx = m_K(0, 0);
//        double fy = m_K(1, 1);
//        double X = camPoint[0];
//        double Y = camPoint[1];
//        double Z = camPoint[2];
//        double ZInv = 1.0 / (Z + 1e-18);
//        double ZInv2 = ZInv * ZInv;
//
//        _jacobianOplusXi << -fx * ZInv, 0, fx * X * ZInv2, fx * X * Y * ZInv2,
//                -fx - fx * X * X * ZInv2, fx * Y * ZInv, 0, -fy * ZInv,
//                fy * Y * ZInv2, fy + fy * Y * Y * ZInv2, -fy * X * Y * ZInv2,
//                -fy * X * ZInv;
//    }

    void EdgePoseOnly::computeError()
    {
        const VertexPose *vertexPose = dynamic_cast<VertexPose *>(_vertices[0]);
        const Sophus::SE3d &Tcw = vertexPose->estimate();
        Eigen::Vector3d pixelPoint = m_K * (Tcw * m_mapPoint);
        pixelPoint /= pixelPoint[2];
        _error = _measurement - pixelPoint.head<2>();
    }

    void EdgePosePoint::computeError()
    {
        const VertexPose *vertexPose = dynamic_cast<VertexPose *>(_vertices[0]);
        const VertexXYZ *vertexXyz = dynamic_cast<VertexXYZ *>(_vertices[1]);
        const Sophus::SE3d &Tcw = vertexPose->estimate();
        const Eigen::Vector3d &mapPoint = vertexXyz->estimate();

        Eigen::Vector3d pixelPoint = m_K * (Tcw * mapPoint);
        pixelPoint /= pixelPoint[2];
        _error = _measurement - pixelPoint.head<2>();
    }

    void EdgePosePoint::linearizeOplus()
    {
        const VertexPose *vertexPose = dynamic_cast<VertexPose *>(_vertices[0]);
        const VertexXYZ *vertexXyz = dynamic_cast<VertexXYZ *>(_vertices[1]);
        const Sophus::SE3d &Tcw = vertexPose->estimate();
        const Eigen::Vector3d &mapPoint = vertexXyz->estimate();
        const Eigen::Vector3d camPoint = Tcw * mapPoint;

        double fx = m_K(0, 0);
        double fy = m_K(1, 1);
        double X = camPoint[0];
        double Y = camPoint[1];
        double Z = camPoint[2];
        double ZInv = 1.0 / (Z + 1e-18);
        double ZInv2 = ZInv * ZInv;

        _jacobianOplusXi << -fx * ZInv, 0, -fx * X * ZInv2,
                -fx * X * Y * ZInv2, fx + fx * X * X / ZInv2
                , -fx * Y * ZInv, 0, fy * ZInv, -fy * Y * ZInv2,
                -fy - fy * Y * Y * ZInv2, fy * X * Y * ZInv2, fy * X * ZInv;

        _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * Tcw.rotationMatrix();
    }
} // myslam