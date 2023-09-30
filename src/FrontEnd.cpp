//
// Created by rookie-lu on 23-9-25.
//
#include <cmath>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <fmt/format.h>

#include "myslam/FrontEnd.h"
#include "myslam/G2OTypes.h"

namespace myslam
{
    FrontEnd::Ptr FrontEnd::getInstance()
    {
        static std::shared_ptr<FrontEnd> frontEnd(new FrontEnd);
        return frontEnd;
    }

    std::vector<MapPoint::Ptr> FrontEnd::initKeyFrame(Frame::Ptr &frame)
    {
        frame->initFrame();
        const auto &leftFeatures = frame->getLeftFeatures();
        const auto &rightFeatures = frame->getRightFeatures();

        std::vector<MapPoint::Ptr> mapPoints;
        for (int idx = 0; idx < leftFeatures.size(); ++idx) {
            Eigen::Vector3d framePoint = triangulate(leftFeatures[idx]->position(), rightFeatures[idx]->position());
            Eigen::Vector3d worldPoint = frame->pose().inverse() * framePoint;
            MapPoint::Ptr mapPoint = MapPoint::createMapPoint();
            mapPoint->setPosition(std::move(worldPoint));
            mapPoints.push_back(mapPoint);
        }
        frame->linkMapPoint(mapPoints);
        return mapPoints;
    }

    Eigen::Vector3d FrontEnd::triangulate(const Eigen::Vector2d &refPoint, const Eigen::Vector2d &currPoint)
    {
        Eigen::Vector3d refNormPoint;
        Eigen::Vector3d currNormPoint;
        Eigen::Vector3d baseline(m_camera->baseline(), 0, 0);

        m_camera->pixel2norm(refPoint, refNormPoint);
        m_camera->pixel2norm(currPoint, currNormPoint);
        double refNormPointNorm = refNormPoint.norm();
        double currNormPointNorm = currNormPoint.norm();

        double cosGamma = refNormPoint.dot(currNormPoint) / (refNormPointNorm * currNormPointNorm);
        double cosBeta = currNormPoint.dot(-baseline) / (currNormPointNorm * m_camera->baseline());
        double gamma = std::acos(cosGamma);
        double beta = std::acos(cosBeta);
        double length = m_camera->baseline() * std::sin(beta) / std::sin(gamma);

        return refNormPoint.normalized() * length;
    }

    void FrontEnd::track()
    {
        const auto &refFeatures = m_refFrame->getLeftFeatures();
        m_currFrame->getLeftFeatures().clear();
        m_currFrame->getLeftFeatures().resize(refFeatures.size(), nullptr);
        std::vector<cv::Point2f> refPoints, currPoints;
        std::vector<uchar> status;
        std::vector<float> err;
        std::vector<std::size_t> fillInIds;

        for (std::size_t idx = 0; idx < refFeatures.size(); ++idx) {
            const auto &feature = refFeatures[idx];
            if (feature == nullptr || feature->isOutlier())
                continue;
            refPoints.emplace_back(feature->position().x(), feature->position().y());
            fillInIds.push_back(idx);
        }

        cv::calcOpticalFlowPyrLK(m_refFrame->getLeftIMG(), m_currFrame->getLeftIMG(),
                                 refPoints, currPoints, status, err);

        std::size_t featureNum = 0;
        for (std::size_t idx = 0; idx < status.size(); ++idx) {
            if (status[idx] == 0) {
                m_currFrame->getLeftFeatures()[fillInIds[idx]] = nullptr;
            } else {
                Feature::Ptr feature = std::make_shared<Feature>(
                        Eigen::Vector2d(currPoints[idx].x, currPoints[idx].y));
                feature->setMapPoint(refFeatures[fillInIds[idx]]->mapPoint().lock());
                m_currFrame->getLeftFeatures()[fillInIds[idx]] = feature;

                ++featureNum;
            }
        }
    }

    bool FrontEnd::poseEstimate(Sophus::SE3d &firstOptiRet)
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

        g2o::SparseOptimizer graph;
        graph.setAlgorithm(solver);

        // construct camera Pose Vertex
        auto *vertexPose = new VertexPose;
        vertexPose->setId(0);
        vertexPose->setEstimate(m_currFrame->pose());
        graph.addVertex(vertexPose);

        // construct edge
        int edgeID = 1;
        const auto &leftFeatures = m_currFrame->getLeftFeatures();
        std::vector<EdgePoseOnly *> edges;
        std::vector<Feature::Ptr> features;

        for (const auto &feature: leftFeatures) {
            if (feature == nullptr) continue;
            const auto &mapPoint = feature->mapPoint().lock();
            if (mapPoint == nullptr) continue;

            features.push_back(feature);
            auto *edge = new EdgePoseOnly(mapPoint->position(), m_camera->K());
            edge->setId(edgeID++);
            edge->setVertex(0, vertexPose);
            edge->setMeasurement(feature->position());
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            graph.addEdge(edge);
            edges.push_back(edge);
        }

        int outlierNum = outlierDeter(vertexPose, graph, edges, features, firstOptiRet);
        int inlierNum = (int) features.size() - outlierNum;
        if (inlierNum > 20) {
            m_currFrame->pose() = vertexPose->estimate();
            std::cout << "m_currFrame的id为： " << m_currFrame->getFrameID() << std::endl;
            std::cout << vertexPose->estimate().matrix() << std::endl;
            return true;
        }
        return false;
    }

    // outlier point determination
    int FrontEnd::outlierDeter(VertexPose *vertexPose, g2o::SparseOptimizer &graph, std::vector<EdgePoseOnly *> &edges,
                               std::vector<Feature::Ptr> &features, Sophus::SE3d &firstOptiRet)
    {
        const double chi2Th = 5.991;  // todo: 将5.991写入config文件
        int outlierNum = 0;
        for (int iteration = 0; iteration < 4; ++iteration)  // todo: 将4写入config文件
        {
            outlierNum = 0;
            vertexPose->setEstimate(m_currFrame->pose());
            graph.initializeOptimization();
            graph.optimize(10); // todo: 将10写入config文件
            if (iteration == 0) firstOptiRet = vertexPose->estimate();

            for (int idx = 0; idx < edges.size(); ++idx) {
                auto &edge = edges[idx];
                auto &feature = features[idx];
                if (feature->isOutlier()) {
                    edge->computeError();
                }
                if (edge->chi2() < chi2Th) {
                    feature->setOutlier(false);
                    edge->setLevel(0);
                } else {
                    feature->setOutlier(true);
                    edge->setLevel(1);
                    ++outlierNum;
                }
                if (iteration == 2) {
                    edge->setRobustKernel(nullptr);
                }
            }
        }
        return outlierNum;
    }

    void FrontEnd::run()
    {
        Frame::Ptr firstFrame = m_dataset->nextFrame();
        auto mapPoints = initKeyFrame(firstFrame);
        m_map->insertKeyFrame(firstFrame);
        m_map->insertMapPoints(mapPoints);

        m_refFrame = firstFrame;
        bool currFramePoseEstimateRet = true;

        while (true) {
            Sophus::SE3d firstOptiRet;
            if (currFramePoseEstimateRet)
                m_currFrame = m_dataset->nextFrame();
            if (m_currFrame == nullptr) break;
            track();
            currFramePoseEstimateRet = poseEstimate(firstOptiRet);
            if (currFramePoseEstimateRet) {
                m_viewer->setFrame(m_currFrame);
                m_refFrame = m_currFrame;
            } else {
                static std::size_t lastFalseTrackID = 0;
                if (lastFalseTrackID == m_refFrame->getFrameID()) {
                    std::cout << "Frame: " << lastFalseTrackID << "重复track失败" << std::endl;
                    // m_currFrame = m_dataset->nextFrame();
                    m_currFrame->pose() = firstOptiRet;
                    m_viewer->setFrame(m_currFrame);

                    auto &features = m_currFrame->getLeftFeatures();
                    for (auto &feature: features) {
                        if (feature == nullptr) continue;
                        feature->setOutlier(false);
                    }
                    m_refFrame = m_currFrame;
                    currFramePoseEstimateRet = true;

                } else {
                    mapPoints = initKeyFrame(m_refFrame);
                    m_map->insertKeyFrame(m_refFrame);
                    m_map->insertMapPoints(mapPoints);
                    lastFalseTrackID = m_refFrame->getFrameID();
                }
            }
        }
    }
} // myslam