//
// Created by rookie-lu on 23-9-25.
//

#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H

#include <string>
#include <memory>
#include <Eigen/Dense>

#include "myslam/Frame.h"
#include "myslam/Camera.h"

namespace myslam
{
    class Dataset
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Dataset> Ptr;

        explicit Dataset(std::string filePath)
                : m_prefix(std::move(filePath))
        {}

        Dataset(const Dataset &) = delete;

        Dataset &operator=(const Dataset &) = delete;

        Frame::Ptr nextFrame();

        Camera::Ptr createCamera();

    private:
        std::string m_prefix;
    };

} // myslam

#endif //MYSLAM_DATASET_H
