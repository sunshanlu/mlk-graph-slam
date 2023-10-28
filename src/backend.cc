#include "backend.h"

NAMESPACE_BEGIN

/**
 * @brief   进行优化，外点剔除，优化结果回带和清理地图信息
 * @param   [in]lck:在激活地图点和激活关键帧拷贝完成后释放互斥量
 *
 */
void Backend::optimize(std::unique_lock<std::mutex> &lck) {
    std::vector<Frame::Ptr> frames;
    std::vector<MapPoint::Ptr> points;
    getPointsAndFrames(frames, points);
    lck.unlock();
    g2o::SparseOptimizer graph;
    auto *algorithm = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    graph.setAlgorithm(algorithm);

    std::unordered_map<std::size_t, PoseVertex *> poseVertexes;
    std::unordered_map<std::size_t, PointVertex *> pointVertexes;
    std::unordered_map<Feature::Ptr, PosePointEdge *> edges;

    addVertexAndEdge(poseVertexes, pointVertexes, edges, frames, points, graph);
    graph.initializeOptimization();
    graph.optimize(Config::OPTIMIZE_ITERATIONS);
    double ret = detectOutlier(graph, edges);
    SPDLOG_INFO("后端优化完成，inliner数目占比为{}", ret);

    setOptValue(poseVertexes, pointVertexes);
    m_viewer->updatePoints(std::move(points));
}

/**
 * @brief   后端启动的线程函数
 *
 */
void Backend::run() {
    while (!m_isStop && !m_viewer->getFlag()) {
        std::unique_lock<std::mutex> lck(m_map->m_mutex);
        m_map->m_cond.wait(lck, [this]() { return this->m_map->m_isReady; });
        optimize(lck);
        m_map->m_isReady = false;
    }
    m_isStop = true;
}

/**
 * @brief   从m_map中获取激活关键帧和激活地图点
 * @param   [in,out]frames: 保存关键帧的指针
 * @param   [in,out]points: 保存地图点的指针
 *
 */
void Backend::getPointsAndFrames(std::vector<Frame::Ptr> &frames,
                                 std::vector<MapPoint::Ptr> &points) {
    for (const auto &frame : m_map->getActiveFrames())
        frames.push_back(frame.second);
    for (const auto &point : m_map->getActivePoints())
        points.push_back(point.second);
}

/**
 * @brief   添加顶点和边信息
 * @param   [in,out]poseVertexes:向其中添加位姿顶点指针
 * @param   [in,out]pointVertexes:向其中添加地图顶点指针
 * @param   [in,out]edges:向其中添加优化边指针
 * @param   [in]frames:地图中的激活关键帧
 * @param   [in]points:地图中的激活地图点
 * @param   [in,out]graph:g2o::SparseOptimizer类型
 *
 */
void Backend::addVertexAndEdge(std::unordered_map<std::size_t, PoseVertex *> &poseVertexes,
                               std::unordered_map<std::size_t, PointVertex *> &pointVertexes,
                               std::unordered_map<Feature::Ptr, PosePointEdge *> &edges,
                               std::vector<Frame::Ptr> &frames, std::vector<MapPoint::Ptr> &points,
                               g2o::SparseOptimizer &graph) {
    // 激活关键帧Vertx
    int vertexID = 0;
    for (auto &frame : frames) {
        auto *poseVertex = new PoseVertex;
        poseVertex->setId(vertexID++);
        poseVertex->setEstimate(frame->pose());
        graph.addVertex(poseVertex);
        poseVertexes.insert(std::make_pair(frame->m_keyID, poseVertex));
    }
    // 激活地图点Vertx
    for (auto &point : points) {
        auto *pointVertex = new PointVertex;
        pointVertex->setId(vertexID++);
        pointVertex->setEstimate(point->m_position);
        pointVertex->setMarginalized(true);
        graph.addVertex(pointVertex);
        pointVertexes.insert(std::make_pair(point->m_id, pointVertex));
    }

    // 构造edge
    int edgeID = 0;
    for (auto &frame : frames) {
        auto poseVertex = poseVertexes.at(frame->m_keyID); // 找不到会出现异常
        for (auto &feature : frame->getLeftFeatures()) {
            auto mapPoint = feature->m_mapPoint.lock();
            if (mapPoint == nullptr)
                continue;
            auto pointVertex = pointVertexes.at(mapPoint->m_id); // 找不到会出现异常
            auto edge = new PosePointEdge(m_camera->m_K);
            auto *kernel = new g2o::RobustKernelHuber;
            kernel->setDelta(Config::EDGE_CHI2_TH);
            edge->setId(edgeID++);
            edge->setVertex(0, poseVertex);
            edge->setVertex(1, pointVertex);
            edge->setMeasurement(feature->m_position);
            edge->setInformation(Mat2d::Identity());
            edge->setRobustKernel(kernel);
            graph.addEdge(edge);
            edges.insert(std::make_pair(feature, edge));
        }
    }
}

/**
 * @brief   检测后端优化过程中的外点
 * @param   [in]graph:g2o::SparseOptimizer对象
 * @param   [in]edges:构造的优化边
 *
 * @note    注意，由于后端优化过程比较长，因此不会采用类似前端的外点剔除方法
 *
 */
double Backend::detectOutlier(g2o::SparseOptimizer &graph,
                              std::unordered_map<Feature::Ptr, PosePointEdge *> &edges) {
    // outlier点检测
    int outlierNum = 0, inlierNum = 0;
    double edgeChTh = Config::EDGE_CHI2_TH;
    for (int iteration = 0; iteration < Config::OUTLIER_DETECT_ITERATIONS; ++iteration) {
        outlierNum = 0;
        inlierNum = 0;
        for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
            MapPoint::Ptr point = iter->first->m_mapPoint.lock();
            assert(point != nullptr);
            PosePointEdge *edge = iter->second;
            if (edge->chi2() > edgeChTh)
                ++outlierNum;
            else
                ++inlierNum;
        }
        if (inlierNum / (outlierNum + inlierNum) > 0.5)
            break;
        edgeChTh *= 1.1;
    }

    // 进行外点剔除（移除Feature和MapPoint之间的关系）
    for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
        Feature::Ptr feature = iter->first;
        MapPoint::Ptr point = feature->m_mapPoint.lock();
        PosePointEdge *edge = iter->second;
        assert(point != nullptr);
        if (edge->chi2() > edgeChTh) {
            feature->m_mapPoint.reset();
            point->m_featureNum--;
        }
    }
    return inlierNum / (outlierNum + inlierNum);
}

/**
 * @brief   保证地图的线程安全条件下，更新地图信息
 * @param   [in]poseVertexes:位姿顶点的unordered_map
 * @param   [in]pointVertexes:地图点的unordered_map
 *
 */
void Backend::setOptValue(std::unordered_map<std::size_t, PoseVertex *> poseVertexes,
                          std::unordered_map<std::size_t, PointVertex *> pointVertexes) {
    std::unique_lock<std::mutex> lck(m_map->m_mutex);
    auto frames = m_map->getAllFrames();
    auto points = m_map->getAllPoints();
    for (auto &poseVertex : poseVertexes) {
        auto &frame = frames.at(poseVertex.first);
        frame->pose() = poseVertex.second->estimate();
    }
    for (auto &pointVertex : pointVertexes) {
        auto &point = points.at(pointVertex.first);
        point->m_position = pointVertex.second->estimate();
    }
    m_map->cleanMap();
}

NAMESPACE_END