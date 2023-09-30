//
// Created by rookie-lu on 23-9-25.
//

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include <fmt/format.h>
#include <iostream>
#include <fstream>
#include <memory>
#include <utility>

#include <Eigen/Dense>

namespace myslam
{

    class Camera
    {
    public:
        typedef std::shared_ptr<Camera> Ptr;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Camera(Eigen::Matrix3d K, double baseline)
                : m_K(std::move(K)), m_baseline(baseline)
        {}

        void camera2pixel(const Eigen::Vector3d &camera, Eigen::Vector2d &pixel);

        void pixel2norm(const Eigen::Vector2d &pixel, Eigen::Vector3d &norm);

        double baseline() const
        { return m_baseline; }

        const Eigen::Matrix3d &K() const
        { return m_K; }




    private:
        Eigen::Matrix3d m_K; // 相机内参
        double m_baseline = 0.0;
    };

} // myslam

#endif //MYSLAM_CAMERA_H
