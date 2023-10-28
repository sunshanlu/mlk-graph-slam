#include "frontend.h"

NAMESPACE_BEGIN

void printPose(const SE3d &pose) {
    Mat4d poseMat = pose.matrix();
    for (int i = 0; i < 4; ++i) {
        SPDLOG_INFO("{:<07.2f}  {:<07.2f}  {:<07.2f}  {:<07.2f}", poseMat(i, 0), poseMat(i, 1),
                    poseMat(i, 2), poseMat(i, 3));
    }
}

/**
 * @brief   前端启动线程
 */
void Frontend::run() {
    while (!m_isStop && !m_viewer->getFlag()) {
        auto frame = m_dataset->createFrame();
        if (frame == nullptr) {
            m_isStop = true;
            break;
        }
        addFrame(frame);
    }
    m_viewer->setFlag();
    /// @note 需要考虑在关闭前端后，后端一直wait的状态
    m_map->m_isReady = true;
    m_map->m_cond.notify_one();
}

void Frontend::addFrame(const Frame::Ptr &frame) {
    m_currFrame = frame;
    switch (m_status) {
    case TrackStatus::INIT_TRACKING:
        initTrack();
        break;
    case TrackStatus::TRACK_GOOD:
    case TrackStatus::TRACK_BAD:
        track();
        break;
    case TrackStatus::TRACK_LOST:
        throw std::runtime_error("this is no implement");
        break;
    }
    if (m_viewer)
        m_viewer->updateFrame(m_currFrame);
    m_refFrame = m_currFrame;
}

/**
 * @brief           根据前端追踪的结果进行位姿估计和异常值检测
 * @return          int 返回m_currFrame关联的MapPoint非异常值的数量
 *
 */
int Frontend::estimatePose() {
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    Mat3d K = m_camera->m_K;
    std::vector<PoseEdge *> edges;
    std::vector<Feature::Ptr> features;

    // pose vertex
    auto vertex = new PoseVertex;
    vertex->setId(0);
    optimizer.addVertex(vertex);

    // construct edge
    int idx = 1;
    for (auto &feature : m_currFrame->getLeftFeatures()) {
        auto mapPoint = feature->m_mapPoint.lock();
        if (mapPoint == nullptr)
            continue;
        auto edge = new PoseEdge(mapPoint->m_position, K);
        edge->setId(idx++);
        edge->setVertex(0, vertex);
        edge->setInformation(Mat2d::Identity());
        edge->setMeasurement(feature->m_position);
        edge->setRobustKernel(new g2o::RobustKernelHuber);
        edges.push_back(edge);
        features.push_back(feature);
        optimizer.addEdge(edge);
    }
    int outlierNum = detectOutlier(features, edges, optimizer, vertex);
    int inlierNum = edges.size() - outlierNum;
    // set track status
    if (inlierNum > Config::TRACK_GOOD_TH)
        m_status = TrackStatus::TRACK_GOOD;
    else if (inlierNum > Config::TRACK_BAD_TH)
        m_status = TrackStatus::TRACK_BAD;
    else
        m_status = TrackStatus::TRACK_LOST;

    return inlierNum;
}

int Frontend::detectOutlier(std::vector<Feature::Ptr> &features, std::vector<PoseEdge *> &edges,
                            g2o::SparseOptimizer &optimizer, PoseVertex *vertex) {
    int outlierNum;
    for (int iteration = 0; iteration < Config::OUTLIER_DETECT_ITERATIONS; ++iteration) {
        outlierNum = 0;
        vertex->setEstimate(m_currFrame->pose());
        bool ret = optimizer.initializeOptimization();
        optimizer.optimize(Config::OPTIMIZE_ITERATIONS);

        for (std::size_t idx = 0; idx < edges.size(); ++idx) {
            auto &feature = features[idx];
            auto &edge = edges[idx];
            auto mapPoint = feature->m_mapPoint.lock();
            assert(mapPoint != nullptr);
            if (mapPoint->m_isOutlier)
                edge->computeError();

            if (edge->chi2() > Config::EDGE_CHI2_TH) {
                mapPoint->m_isOutlier = true;
                edge->setLevel(1);
                ++outlierNum;
            } else {
                mapPoint->m_isOutlier = false;
                edge->setLevel(0);
            }

            if (iteration == 2)
                edge->setRobustKernel(nullptr);
        }
    }

    // set pose and outlier
    m_currFrame->pose() = vertex->estimate();
    for (auto &feature : features) {
        auto mapPoint = feature->m_mapPoint.lock();
        assert(mapPoint != nullptr);
        if (mapPoint->m_isOutlier) {
            feature->m_mapPoint.reset();
            /// @note 这里MapPoint的m_isOutlier在这里进行辅助，需要恢复为false
            mapPoint->m_isOutlier = false;
        }
    }
    return outlierNum;
}

/**
 * @brief       为前端开启循环做m_refFrame的准备
 *
 */
void Frontend::initTrack() {
    std::vector<MapPoint::Ptr> newPoints;
    m_currFrame->initKeyFrame(m_camera);
    int frameGoodMapPoint = triNewPoint(m_currFrame, newPoints);

    // 初始化关键帧后的输出
    SPDLOG_INFO("初始化关键帧: {:06}\t得到{:03}个地图点", m_currFrame->m_keyID, frameGoodMapPoint);
    {
        assert(m_map != nullptr);
        std::unique_lock<std::mutex> mapLck(m_map->m_mutex);
        m_map->insertMapPoints(newPoints);
        m_map->insertKeyFrame(m_currFrame);
        m_map->m_isReady = true;
        m_map->m_cond.notify_one();
    }
    m_status = TrackStatus::TRACK_GOOD;
    /// @note 注意这里需要线程停滞100us，来等待开启后端的优化线程
    std::this_thread::sleep_for(std::chrono::microseconds(100));
}

/**
 * @brief
 * @param           [in ]poses:        位姿列表
 * @param           [in ]pixelPoints:  像素点列表
 * @param           [out]worldPoint:   三角化计算的结果
 * @return          true    最小奇异值接近0（存在非零解），并且三角化坐标点z值大于零，三角化成功
 * @return          false   最小奇异值不满足条件，三角化失败
 *
 */
bool Frontend::triangulate(const std::vector<SE3d> &poses, const std::vector<Vec2d> &pixelPoints,
                           Vec3d &worldPoint) {
    MatXd A(2 * poses.size(), 4);
    for (int idx = 0; idx < poses.size(); ++idx) {
        const Mat34d T = poses[idx].matrix3x4();
        const Vec2d &pixelPoint = pixelPoints[idx];
        Vec3d normPoint;
        normPoint << pixelPoint.x(), pixelPoint.y(), 1;
        normPoint = m_camera->m_K.inverse() * normPoint;
        Mat14d xnExp = normPoint.x() * T.row(2) - T.row(0);
        Mat14d ynExp = normPoint.y() * T.row(2) - T.row(1);

        A.block<1, 4>(idx * 2, 0) = xnExp;
        A.block<1, 4>(idx * 2 + 1, 0) = ynExp;
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    worldPoint = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();
    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2 && worldPoint.z() > 0)
        return true;
    return false;
}

/**
 * @brief           三角化新的点
 * @details         维护Feature和MapPoint之间的关系
 *                  1. Feature的m_mappoint指向MapPoint
 *                  2. MapPoint的m_featureNum递增
 * @param           [out]newPoints:新三角化的点集合
 * @param           [in and out]frame:需要三角化的帧
 * @return          int 返回三角测量后，Frame中可用的MapPoint数目
 *
 */
int Frontend::triNewPoint(Frame::Ptr &frame, std::vector<MapPoint::Ptr> &newPoints) {
    int frameGoodMapPoint = 0;
    auto &leftFeatures = frame->getLeftFeatures();
    auto &rightFeatures = frame->getRightFeatures();
    ///////////////////////////////////////////////////////////////////////////////////
    // std::ofstream ifs("/home/rookie-lu/Project/mlk-graph-slam/debug/point_feature.csv");
    // ifs << "left_x"
    //     << ","
    //     << "left_y"
    //     << ","
    //     << "right_x"
    //     << ","
    //     << "right_y"
    //     << ","
    //     << "point_x"
    //     << ","
    //     << "point_y"
    //     << ","
    //     << "point_z" << std::endl;
    ///////////////////////////////////////////////////////////////////////////////////
    for (std::size_t idx = 0; idx < leftFeatures.size(); ++idx) {
        auto &leftFeature = leftFeatures[idx];
        auto &rightFeature = rightFeatures[idx];
        auto mapPoint = leftFeature->m_mapPoint.lock();

        if (mapPoint == nullptr && rightFeature != nullptr) {
            std::vector<Vec2d> pixelPoints;
            pixelPoints.emplace_back(leftFeature->m_position);
            pixelPoints.emplace_back(rightFeature->m_position);
            Vec3d worldPoint;

            bool ret = triangulate(m_camera->m_poses, pixelPoints, worldPoint);
            worldPoint = frame->pose().inverse() * worldPoint;

            if (ret) {
                ///////////////////////////////////////////////////////////////////////////////////
                // ifs << leftFeature->m_position.x() << "," << leftFeature->m_position.y() << ","
                //     << rightFeature->m_position.x() << "," << rightFeature->m_position.y() << ","
                //     << worldPoint.x() << "," << worldPoint.y() << "," << worldPoint.z()
                //     << std::endl;
                ///////////////////////////////////////////////////////////////////////////////////
                auto newPoint = MapPoint::create();
                newPoint->m_position = worldPoint;
                ++(newPoint->m_featureNum);
                leftFeature->m_mapPoint = newPoint;
                newPoints.push_back(newPoint);
                ++frameGoodMapPoint;
            }
        } else if (mapPoint != nullptr) {
            /// @note 将关键帧中可以使用的MapPoint中的m_featureNum增加
            ++(mapPoint->m_featureNum);
            ++frameGoodMapPoint;
        }
    }
    return frameGoodMapPoint;
}

/**
 * @brief           追踪m_refFrame中的特征点到m_currFrame
 * @note            track部分并不维护Feature与MapPoint的关系（非关键帧）
 *
 * @return          int 返回track成功的二维点数量
 *
 */
int Frontend::trackCurrFrame() {
    /// @note Frontend维护m_diffPose来给当前帧追踪赋初值
    m_currFrame->pose() = m_diffPose * m_refFrame->pose();
    auto &features = m_refFrame->getLeftFeatures();
    std::vector<cv::Point2f> refPixelPoints, currPixelPoints;
    for (auto &feature : features) {
        refPixelPoints.emplace_back(feature->m_position.x(), feature->m_position.y());
        auto mapPoint = feature->m_mapPoint.lock();
        if (mapPoint != nullptr) {
            Vec3d worldPoint = mapPoint->m_position;
            worldPoint = m_currFrame->pose() * worldPoint;
            worldPoint = worldPoint / (worldPoint.z() + 1e-18);
            Vec2d pixelPoint = (m_camera->m_K * worldPoint).head<2>();
            currPixelPoints.emplace_back(pixelPoint.x(), pixelPoint.y());
        } else {
            currPixelPoints.emplace_back(feature->m_position.x(), feature->m_position.y());
        }
    }
    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        m_refFrame->m_leftIMG, m_currFrame->m_leftIMG, refPixelPoints, currPixelPoints, status,
        error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int trackSuccess = 0;
    for (std::size_t idx = 0; idx < status.size(); ++idx) {
        auto leftFeature = features[idx];
        auto mapPoint = leftFeature->m_mapPoint.lock();
        if (status[idx]) {
            ++trackSuccess;
            auto rightFeature = Feature::create();
            rightFeature->m_position = Vec2d(currPixelPoints[idx].x, currPixelPoints[idx].y);
            if (mapPoint != nullptr) {
                rightFeature->m_mapPoint = mapPoint;
            }
            m_currFrame->getLeftFeatures().push_back(rightFeature);
        }
    }
    return trackSuccess;
}

NAMESPACE_END