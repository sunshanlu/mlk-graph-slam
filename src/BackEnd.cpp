//
// Created by rookie-lu on 23-9-30.
//
#include <vector>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "myslam/G2OTypes.h"
#include "myslam/BackEnd.h"

namespace myslam
{
    BackEnd::Ptr BackEnd::getInstance()
    {
        static Ptr backend(new BackEnd);
        return backend;
    }

    void BackEnd::run()
    {
        while (!m_stop) {
            // todo: 这里需要使用条件变量进行等待
            optimize();
        }
    }

    void BackEnd::optimize()
    {
        double chi2_th = 5.991;  // todo: 将5.991写入config文件
        typedef std::pair<VertexPose *, std::vector<VertexXYZ *>> VertexRelationType;
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

        static std::function<bool(VertexPose *, VertexPose *)> sortFunc = [](
                VertexPose *leftItem, VertexPose *rightItem) {
            return leftItem->id() < rightItem->id();
        };

        g2o::SparseOptimizer graph;
        graph.setAlgorithm(solver);

        int vertexID = 0, edgeID = 0;
        std::vector<VertexPose *> poseVertexes;
        std::vector<VertexXYZ *> xyzVertexes;

        auto activeKeyFrames = m_map->getActiveFrames();
        std::list<MapPoint::Ptr> activeMapPoints;

        for (const auto &keyFrame: activeKeyFrames) {
            const auto &features = keyFrame->getLeftFeatures();
            auto *vertexPose = new VertexPose;
            vertexPose->setId(vertexID++);
            vertexPose->setEstimate(keyFrame->pose());
            graph.addVertex(vertexPose);
            poseVertexes.push_back(vertexPose);
            std::vector<int> xyzVertexIDS;

            for (const auto &feature: features) {
                if (feature == nullptr || feature->isOutlier()) continue;
                const auto &mapPoint = feature->mapPoint().lock();
                if (mapPoint == nullptr) continue;

                auto *vertexXYZ = new VertexXYZ;
                vertexXYZ->setId(vertexID++);
                vertexXYZ->setEstimate(mapPoint->position());
                vertexXYZ->setMarginalized(true);
                graph.addVertex(vertexXYZ);
                xyzVertexes.push_back(vertexXYZ);
                activeMapPoints.push_back(mapPoint);

                auto *edge = new EdgePosePoint(m_camera->K());
                edge->setId(edgeID++);
                edge->setVertex(0, vertexPose);
                edge->setVertex(1, vertexXYZ);
                edge->setMeasurement(feature->position());
                edge->setInformation(Eigen::Matrix2d::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                graph.addEdge(edge);
            }
        }
        graph.initializeOptimization();
        graph.optimize(10);

        // 将优化后的结果写入类中
        assert(poseVertexes.size() == activeKeyFrames.size());
        auto vertex = poseVertexes.begin();
        auto keyFrame = activeKeyFrames.begin();
        while (true) {
            if (vertex == poseVertexes.end()) break;
            (*keyFrame)->pose() = (*vertex)->estimate();
            ++vertex;
            ++keyFrame;
        }

        // todo: 做mapPoint的清理工作
        m_map->setActivePoints(activeMapPoints);
    }
}



