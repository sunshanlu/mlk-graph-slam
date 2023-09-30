//
// Created by rookie-lu on 23-9-25.
//

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include <vector>

#include <Eigen/Dense>

#include "myslam/Feature.h"

namespace myslam
{
    class MapPoint
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<MapPoint> Ptr;

        MapPoint() = default;

        void setPosition(Eigen::Vector3d position)
        { m_position = std::move(position); }

        static Ptr createMapPoint();

        Eigen::Vector3d &position()
        { return m_position; }

        void setFeature(const Feature::Ptr &feature)
        {
            m_features.push_back(feature);
            ++m_featureNum;
        }

        void removeFeature(const Feature::Ptr &feature)
        {
            for (auto it = m_features.begin(); it != m_features.end(); ++it) {
                if (it->lock() != feature) {
                    continue;
                }
                m_features.erase(it);
                break;
            }
            --m_featureNum;
        }

        std::size_t getFeatureNum() const
        { return m_featureNum; }

    private:
        Eigen::Vector3d m_position;
        std::list<std::weak_ptr<Feature>> m_features;
        std::size_t m_featureNum = 0;
        std::size_t m_ID = -1;
    };

} // myslam

#endif //MYSLAM_MAPPOINT_H
