#include "g2o_types.h"

NAMESPACE_BEGIN

void PoseEdge::computeError() {
    const auto *vertex = dynamic_cast<PoseVertex *>(_vertices[0]);
    SE3d pose = vertex->estimate();
    Vec3d pixelPoint = m_K * (pose * m_point);
    pixelPoint = pixelPoint / pixelPoint.z();
    _error = _measurement - pixelPoint.head<2>();
}

void PoseEdge::linearizeOplus() {
    const PoseVertex *v = static_cast<PoseVertex *>(_vertices[0]);
    SE3d T = v->estimate();
    Vec3d pos_cam = T * m_point;
    double fx = m_K(0, 0);
    double fy = m_K(1, 1);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2, -fx - fx * X * X * Zinv2,
        fx * Y * Zinv, 0, -fy * Zinv, fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
        -fy * X * Zinv;
}

void PosePointEdge::computeError() {
    const auto *poseVertex = dynamic_cast<PoseVertex *>(_vertices[0]);
    const auto *pointVertex = dynamic_cast<PointVertex *>(_vertices[1]);

    SE3d pose = poseVertex->estimate();
    Vec3d pixelPoint = m_K * (pose * pointVertex->estimate());
    pixelPoint = pixelPoint / pixelPoint.z();
    _error = _measurement - pixelPoint.head<2>();
}

void PosePointEdge::linearizeOplus() {
    auto *poseVertex = dynamic_cast<PoseVertex *>(_vertices[0]);
    auto *pointVertex = dynamic_cast<PointVertex *>(_vertices[1]);
    SE3d pose = poseVertex->estimate();
    Vec3d worldPoint = pointVertex->estimate();
    Vec3d cameraPoint = pose * worldPoint;
    double fx = m_K(0, 0);
    double fy = m_K(1, 1);
    double X = cameraPoint[0];
    double Y = cameraPoint[1];
    double Z = cameraPoint[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2, -fx - fx * X * X * Zinv2,
        fx * Y * Zinv, 0, -fy * Zinv, fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
        -fy * X * Zinv;

    _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * pose.rotationMatrix();
}

NAMESPACE_END