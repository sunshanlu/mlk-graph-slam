//
// Created by rookie-lu on 23-9-25.
//

#include "myslam/Camera.h"

namespace myslam
{
    void Camera::camera2pixel(const Eigen::Vector3d &camera, Eigen::Vector2d &pixel)
    {
        Eigen::Vector3d point = m_K * camera / camera.z();
        pixel.x() = point.x();
        pixel.y() = point.y();
    }

    void Camera::pixel2norm(const Eigen::Vector2d &pixel, Eigen::Vector3d &norm)
    {
        Eigen::Vector3d point(pixel.x(), pixel.y(), 1);
        norm = m_K.inverse() * point;
    }
} // myslam