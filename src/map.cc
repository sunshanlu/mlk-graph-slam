#include "map.h"

NAMESPACE_BEGIN

/**
 * @brief           设定关键帧，并进行特征检测、匹配
 * @details         当前端位姿估计后，Frontend::m_currFrame中
 *                  关联的MapPoint少于下次位姿估计所需的地图点时
 *                  就需要进行特征检测和匹配
 * @note            设定GFTT最近的距离为20
 * @return          int 进行关键帧初始化后，Frame中可用"特征对"的个数
 *
 */
int Frame::initKeyFrame(const Camera::Ptr &camera) {
    static int keyFrameNum = 0;
    m_isKeyFrame = true;
    m_keyID = keyFrameNum++;

    // 1. 略过左图已经存在的特征点(创建mask)
    cv::Mat mask(m_leftIMG.size(), CV_8UC1, 255);
    for (auto &feature : m_leftFeatures) {
        cv::Point2f position(feature->m_position.x(), feature->m_position.y());
        cv::rectangle(mask, position - cv::Point2f(10, 10), position + cv::Point2f(10, 10), 0,
                      CV_FILLED);
    }

    // 2. 进行特征点检测(GFTT)
    std::vector<cv::KeyPoint> leftKeyPoints;
    auto gftt = cv::GFTTDetector::create(Config::DETECT_FEATURE_NUM, 0.01, 20);
    gftt->detect(m_leftIMG, leftKeyPoints);
    for (const auto &point : leftKeyPoints) {
        double xVal = point.pt.x;
        double yVal = point.pt.y;
        auto feature = Feature::create();
        feature->m_position = Vec2d(xVal, yVal);
        m_leftFeatures.push_back(feature);
    }

    // 3. 设定右图的初始值，并进行LK光流追踪
    std::vector<cv::Point2f> leftPoints, rightPoints;
    for (const auto &feature : m_leftFeatures) {
        float xVal = feature->m_position.x();
        float yVal = feature->m_position.y();
        leftPoints.emplace_back(xVal, yVal);
    }
    assert(m_leftFeatures.size() == leftPoints.size());
    for (std::size_t idx = 0; idx < leftPoints.size(); ++idx) {
        auto mapPoint = m_leftFeatures[idx]->m_mapPoint.lock();
        if (mapPoint) {
            Vec3d Pc2 = camera->m_poses[1] * m_pose * mapPoint->m_position;
            Vec3d Pn2 = Pc2 / (Pc2[2] + 1e-18);
            Vec2d pixelPc2 = (camera->m_K * Pn2).head<2>();
            rightPoints.emplace_back(pixelPc2.x(), pixelPc2.y());
        } else
            rightPoints.emplace_back(leftPoints[idx].x, leftPoints[idx].y);
    }
    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        m_leftIMG, m_rightIMG, leftPoints, rightPoints, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    // 4. 根据status设定右图信息
    int goodFeatureNum;
    for (std::size_t idx = 0; idx < status.size(); ++idx) {
        if (status[idx]) {
            double xVal = rightPoints[idx].x;
            double yVal = rightPoints[idx].y;
            auto feature = Feature::create();
            feature->m_position = Vec2d(xVal, yVal);
            m_rightFeatures.push_back(feature);
            ++goodFeatureNum;
        } else {
            m_rightFeatures.push_back(nullptr);
        }
    }

    return goodFeatureNum;
}

/**
 * @brief           计算两个位姿之间的距离，以left为参考
 * @param           [in ]left:第一个位姿（参考位姿）
 * @param           [in ]right:第二个位姿
 * @return          double  计算left和right之间的位姿距离
 *
 */
double Frame::distance(const Frame &left, const Frame &right) {
    SE3d deltaPose = left.m_pose * right.m_pose.inverse();
    Vec6d disVec = deltaPose.log();
    return disVec.lpNorm<2>();
}

void Map::removeOldFrame(const Frame::Ptr &keyFrame) {
    Frame::Ptr frameRemove = nullptr;
    double minVal, maxVal;
    std::size_t minValId, maxValId;
    std::size_t idx = 0;
    for (auto iter = m_activeFrames.begin(); iter != m_activeFrames.end(); ++iter) {
        double disVal = Frame::distance(*keyFrame, *iter->second);
        if (0 == idx) {
            minVal = disVal;
            maxVal = disVal;
            minValId = iter->first;
            maxValId = iter->first;
            continue;
        }
        if (disVal < minVal) {
            minVal = disVal;
            minValId = iter->first;
        } else if (disVal > maxVal) {
            maxVal = disVal;
            maxValId = iter->first;
        }
        ++idx;
    }
    /// @note 保证激活的关键帧的位姿之间，时间连续，空间展开
    if (minVal < Config::KEYFRAME_DISTANCE_TH)
        frameRemove = m_activeFrames.at(minValId);
    else
        frameRemove = m_activeFrames.at(maxValId);
    m_activeFrames.erase(frameRemove->m_keyID);
    SPDLOG_INFO("删除的关键帧的keyID为: {:06}\n", frameRemove->m_keyID);
}

/**
 * @brief 当没有Feature指向MapPoint时，删除MapPoint
 * @note  在异常点剔除之后使用（后端优化后）
 */
void Map::cleanMap() {
    std::vector<std::size_t> removeVec;
    for (auto &point : m_points) {
        assert(point.second->m_featureNum >= 0); // 断言地图点的被指向数目总是大于等于零的
        if (point.second->m_featureNum == 0)
            removeVec.push_back(point.first);
    }
    for (const auto &idx : removeVec) {
        m_points.erase(idx);
        auto iter = m_activePoints.find(idx);
        if (iter != m_activePoints.end())
            m_activePoints.erase(idx);
    }
}

void Map::insertKeyFrame(const Frame::Ptr &keyFrame) {
    assert(keyFrame->m_isKeyFrame == true);
    m_frames.insert(std::make_pair(keyFrame->m_keyID, keyFrame));
    if (m_activeFrames.size() == 7)
        removeOldFrame(keyFrame);
    m_activeFrames.insert(std::make_pair(keyFrame->m_keyID, keyFrame));

    m_activePoints.clear();
    for (auto &frame : m_activeFrames) {
        auto &leftFeatures = frame.second->getLeftFeatures();
        for (auto &feature : leftFeatures) {
            auto mapPoint = feature->m_mapPoint.lock();
            if (mapPoint == nullptr)
                continue;
            m_activePoints.insert(std::make_pair(mapPoint->m_id, mapPoint));
        }
    }
}

NAMESPACE_END