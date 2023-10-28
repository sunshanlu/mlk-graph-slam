/**
 * @file        common_include.h
 * @brief       引入所有所需头文件，定义所有所需超参数
 * @author      rookie-lu (ssl2001@126.com)
 * @version     1.0
 * @date        Oct-28-2023
 *
 * @copyright   Copyright (c) 2023 rookie-lu
 *
 */
#pragma once

#include <chrono>
#include <condition_variable>
#include <fstream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <fmt/format.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <json/json.h>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <spdlog/spdlog.h>

#define NAMESPACE_BEGIN namespace myslam {
#define NAMESPACE_END }
#define USE_EIGEN

NAMESPACE_BEGIN

typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Matrix4d Vec4d;
typedef Eigen::Matrix<double, 6, 1> Vec6d;
typedef Eigen::Matrix3d Mat3d;
typedef Eigen::Matrix4d Mat4d;
typedef Eigen::Matrix4f Mat4f;
typedef Eigen::Matrix2d Mat2d;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXd;
typedef Eigen::Matrix<double, 3, 4> Mat34d;
typedef Eigen::Matrix<double, 1, 4> Mat14d;
typedef Sophus::SE3d SE3d;
typedef Sophus::SO3d SO3d;
typedef g2o::BlockSolver_6_3 BlockSolverType;
typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

NAMESPACE_END