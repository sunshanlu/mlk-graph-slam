//
// Created by rookie-lu on 23-9-25.
//

#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include <vector>
#include <queue>
#include <mutex>

#include "myslam/Frame.h"
#include "myslam/MapPoint.h"

namespace myslam
{

    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Map> Ptr;

        static Ptr getInstance();

        Map(const Map &) = delete;

        Map &operator=(const Map &) = delete;

        Frame::Ptr insertKeyFrame(const Frame::Ptr &frame);

        void insertMapPoints(const std::vector<MapPoint::Ptr> &mapPoints);

        /*
         * 关键帧保证时间上可以连续，空间上可以展开
         * 1. 求可以使关键帧与当前帧空间最小的关键帧
         * 2. 求可以使关键帧与当前帧空间最大的关键帧
         * 3. 设置某个阈值，如果空间距离小于阈值，则删除1，否则删除2
         * */
        Frame::Ptr removeOldKeyFrame(const Frame::Ptr &frame);

        void cleanMap();

        std::list<Frame::Ptr> getActiveFrames()
        {
            std::unique_lock<std::mutex> frameLck(m_frameMutex);
            return m_activeFrames;
        }

        std::list<MapPoint::Ptr> getActivePoints()
        {
            std::unique_lock<std::mutex> pointLck(m_pointMutex);
            return m_activePoints;
        }

        void setActivePoints(std::list<MapPoint::Ptr> mapPoints)
        {
            std::unique_lock<std::mutex> poinLck(m_pointMutex);
            m_activePoints = std::move(mapPoints);
        }


    private:

        Map() = default;

        static double distance(const Frame::Ptr &frame1, const Frame::Ptr &frame2);

        std::vector<Frame::Ptr> m_allFrames;
        std::vector<MapPoint::Ptr> m_allPoints;

        std::list<Frame::Ptr> m_activeFrames;
        std::list<MapPoint::Ptr> m_activePoints;

        std::mutex m_frameMutex;
        std::mutex m_pointMutex;
    };

} // myslam

#endif //MYSLAM_MAP_H
