/**
 * @file        dataset.h
 * @brief       定义Dataset数据集类
 * @author      rookie-lu (ssl2001@126.com)
 * @version     1.0
 * @date        Oct-28-2023
 *
 * @copyright   Copyright (c) 2023 rookie-lu
 *
 */
#pragma once
#include "camera.h"
#include "common_include.h"
#include "map.h"
#include "config.h"

NAMESPACE_BEGIN

/**
 * @brief           Dataset作为数据集的抽象，提供扩展接口
 * @details         继承Dataset的对象必须重写
 *                  createFrame()和createCamera()函数
 *                  Dataset不能拷贝
 * @attention       createCamera()部分的扩展还没有想好
 */
class Dataset {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Dataset> Ptr;

    Dataset(const Dataset &other) = delete;

    Dataset &operator=(const Dataset &other) = delete;

    virtual Frame::Ptr createFrame() = 0;

    virtual Camera::Ptr createCamera() = 0;

    virtual ~Dataset() {}

protected:
    Dataset() = default;
};

/**
 * @brief           KITTISet是Frame和Camera的简单工厂
 * @note            KITTISet采用单例设计模式
 */
class KITTISet : public Dataset {
public:
    /**
     * @brief           单例设计模式（懒汉模式）
     * @return          Ptr 指向全局唯一KITTISet的shared_ptr
     *
     */
    static Ptr getInstance() {
        static Ptr dataset(new KITTISet);
        return dataset;
    }

    Frame::Ptr createFrame() override;

    Camera::Ptr createCamera() override;

private:
    KITTISet() = default;
    std::size_t m_frameNum = 0;
};

NAMESPACE_END