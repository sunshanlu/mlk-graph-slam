/**
 * @file        app.h
 * @brief       保证三个独立线程的初始化和启动
 * @author      rookie-lu (ssl2001@126.com)
 * @version     1.0
 * @date        Oct-27-2023
 *
 * @copyright   Copyright (c) 2023 rookie-lu
 *
 */
#pragma once

#include "backend.h"
#include "camera.h"
#include "common_include.h"
#include "config.h"
#include "dataset.h"
#include "frontend.h"
#include "map.h"
#include "viewer.h"

NAMESPACE_BEGIN

/**
 * @brief   初始化和启动前端，后端和展示线程
 *
 */
class Application {
public:
    typedef std::shared_ptr<Application> Ptr;

    Application(const Application &other) = delete;
    Application &operator=(const Application &other) = delete;

    static Ptr getInstance() {
        static Ptr app(new Application);
        return app;
    }
    /**
     * @brief   启动三个线程
     *
     */
    void run();

private:
    /**
     * @brief   初始化三个线程
     *
     */
    Application();

    Frontend::Ptr m_frontend; /// 前端
    Backend::Ptr m_backend;   /// 后端
    Viewer::Ptr m_viewer;     /// 展示
    Map::Ptr m_map;           /// 地图
    Dataset::Ptr m_dataset;   /// 数据集
    Camera::Ptr m_camera;     /// 相机
};

NAMESPACE_END