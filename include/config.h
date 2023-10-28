/**
 * @file        config.h
 * @brief       编写slam系统的超参数信息类
 * @author      rookie-lu (ssl2001@126.com)
 * @version     1.0
 * @date        Oct-27-2023
 *
 * @copyright   Copyright (c) 2023 rookie-lu
 *
 */
#pragma once

#include "common_include.h"

NAMESPACE_BEGIN

/**
 * @brief   读取超参数并设置超参数
 * @details 读取config文件夹中的config.json文件
 *          并将其中的slam系统运行的参数信息写入
 *          common_include.h头文件中定义的超参数
 *          变量中去
 */

class Config {
public:
    static void loadConfig();

public:
    static std::string KITTI_PATH;          /// KITTI数据集所在目录
    static int DETECT_FEATURE_NUM;          /// 特征检测数目
    static int MAX_ACTIVE_KEYFRAME_NUM;     /// Map中m_activeFrames的数目
    static double KEYFRAME_DISTANCE_TH;     /// 保证Map::m_activeFrames的距离大于0.5
    static double EDGE_CHI2_TH;             /// 优化边的阈值
    static int OUTLIER_DETECT_ITERATIONS;   /// 进行异常点检测的优化迭代次数
    static int OPTIMIZE_ITERATIONS;         /// 单次优化迭代次数
    static int TRACK_GOOD_TH;               /// track good 阈值
    static int TRACK_BAD_TH;                /// track bad 阈值
    static int MIN_MAPPOINTS_NUM_FOR_TRACK; /// track可执行的最少特征数目
    static float MAPPOINT_SIZE;             /// 定义绘制地图点的大小
    static float LINE_WIDTH;                /// 定义绘制线断的大小
    static float CURR_FRAME_COLOR[3];       /// 定义绘制当前帧的颜色
    static float MAPPOINT_COLOR[3];         /// 定义绘制地图点的颜色
    static float MAP_FRAME_COLOR[3];        /// 定义绘制激活关键帧的颜色
    static cv::Scalar FEATURE_COLOR;        /// 定义绘制特征点的颜色
};

NAMESPACE_END