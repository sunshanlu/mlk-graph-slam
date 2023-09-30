//
// Created by rookie-lu on 23-9-25.
//

#include "myslam/MapPoint.h"

namespace myslam
{
    MapPoint::Ptr MapPoint::createMapPoint()
    {
        static std::size_t pointNum = 0;
        Ptr point = std::make_shared<MapPoint>();
        point->m_ID = pointNum++;
        return point;
    }
} // myslam