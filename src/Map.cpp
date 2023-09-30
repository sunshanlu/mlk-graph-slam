//
// Created by rookie-lu on 23-9-25.
//

#include "myslam/Map.h"

namespace myslam
{
    Map::Ptr Map::getInstance()
    {
        static std::shared_ptr<Map> mapPtr(new Map);
        return mapPtr;
    }

    void Map::cleanMap()
    {
        for (auto &point: m_allPoints) {
            if (point->getFeatureNum() != 0)
                continue;
            point = nullptr;
        }
    }

    Frame::Ptr Map::insertKeyFrame(const Frame::Ptr &frame)
    {
        std::unique_lock<std::mutex> frameLck(m_frameMutex);
        m_allFrames.push_back(frame);
        Frame::Ptr popFrame;
        if (m_activeFrames.size() == 7)  // todo: 将7写入config文件
            popFrame = removeOldKeyFrame(frame);
        m_activeFrames.push_back(frame);
        return popFrame;
    }

    Frame::Ptr Map::removeOldKeyFrame(const Frame::Ptr &frame)
    {
        static const double minMaxTh = 0.2;  // todo: 将0.2写入config文件

        double minDistance, maxDistance;
        Frame::Ptr minID, maxID, popFrame;

        auto iter = m_activeFrames.begin();
        Frame::Ptr otherFrame = *iter;
        minDistance = distance(frame, otherFrame);
        maxDistance = minDistance;
        minID = otherFrame;
        maxID = otherFrame;

        for (; iter != m_activeFrames.end(); ++iter) {
            otherFrame = *iter;
            double dis = distance(frame, otherFrame);
            if (dis > maxDistance) {
                maxDistance = dis;
                maxID = otherFrame;
            } else if (dis < minDistance) {
                minDistance = dis;
                minID = otherFrame;
            }
        }
        if (minDistance < 0.2)
            popFrame = minID;
        else
            popFrame = maxID;
        m_activeFrames.remove(popFrame);
        return popFrame;
    }

    double Map::distance(const Frame::Ptr &frame1, const Frame::Ptr &frame2)
    {
        return (frame1->pose() * frame2->pose().inverse()).log().norm();
    }

    void Map::insertMapPoints(const std::vector<MapPoint::Ptr> &mapPoints)
    {
        std::unique_lock<std::mutex> pointLck(m_pointMutex);
        for (const auto &point: mapPoints) {
            m_allPoints.push_back(point);
        }
    }

} // myslam