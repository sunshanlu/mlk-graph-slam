//
// Created by rookie-lu on 23-9-25.
//

#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <memory>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <utility>

namespace myslam
{
    class MapPoint;

    class Feature
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Feature> Ptr;

        Feature() = default;

        explicit Feature(Eigen::Vector2d position)
                : m_position(std::move(position))
        {}

        void setMapPoint(const std::shared_ptr<MapPoint> &mapPoint)
        {
            m_mapPoint = mapPoint;
        }

        std::weak_ptr<MapPoint> &mapPoint()
        { return m_mapPoint; }

        const Eigen::Vector2d &position() const
        {
            return m_position;
        }

        bool isOutlier() const
        { return m_isOutlier; }

        void setOutlier(bool flag)
        { m_isOutlier = flag; }

    private:
        std::weak_ptr<MapPoint> m_mapPoint;
        Eigen::Vector2d m_position;
        bool m_isOutlier = false;
    };

} // myslam

#endif //MYSLAM_FEATURE_H
