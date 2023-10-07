//
// Created by rookie-lu on 23-9-30.
//

#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include <memory>
#include <condition_variable>

#include <Eigen/Dense>

#include "myslam/Map.h"
#include "myslam/Camera.h"

namespace myslam
{
    class BackEnd
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::shared_ptr<BackEnd> Ptr;

        BackEnd(const BackEnd &other) = delete;

        BackEnd &operator=(const BackEnd &other) = delete;

        static Ptr getInstance();

        void run();

        void optimize();

        void setMap(Map::Ptr map)
        { m_map = std::move(map); }

        void setCamera(Camera::Ptr camera)
        { m_camera = std::move(camera); }

    private:
        BackEnd() = default;

        Map::Ptr m_map;

        Camera::Ptr m_camera;

        bool m_stop = false;
    };
}

#endif //MYSLAM_BACKEND_H
