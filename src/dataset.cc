#include "dataset.h"

NAMESPACE_BEGIN

/**
 * @brief           Frame工厂函数
 * @details         createFrame()函数只负责调用Frame的构造函数，
 *                  Frame::m_id由Frame的构造函数负责
 * @note            createFrame()函数将原来的图片进行了0.5倍的缩放
 *                  createFrame()函数采用的缩放算法为INTER_NEAREST
 * @return          Frame::Ptr
 *
 */
Frame::Ptr KITTISet::createFrame() {
    std::string leftPath = fmt::format("{}/image_0/{:06}.png", Config::KITTI_PATH, m_frameNum);
    std::string rightPath = fmt::format("{}/image_1/{:06}.png", Config::KITTI_PATH, m_frameNum++);

    cv::Mat leftResize, rightResize;
    cv::Mat leftIMG = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
    cv::Mat rightIMG = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);
    if (leftIMG.data == nullptr)
        return nullptr;

    cv::resize(leftIMG, leftResize, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
    cv::resize(rightIMG, rightResize, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
    Frame::Ptr frame(new Frame(std::move(leftResize), std::move(rightResize)));
    return frame;
}

/**
 * @brief           Camera工厂函数
 * @details         设定Camera的m_K和m_baseline
 * @note            createFrame()函数部分进行了0.5倍的图像缩放
 *                  createCamera()生成K的前两行缩小了0.5倍以此
 *                  来保证三角测量结果的尺度不变性
 * @return          Camera::Ptr
 *
 */
Camera::Ptr KITTISet::createCamera() {
    std::string camPath = fmt::format("{}/calib.txt", Config::KITTI_PATH);
    std::ifstream camIfs(camPath);
    std::string camName;
    Mat3d K = Mat3d::Zero();
    Vec3d t = Vec3d::Zero();

    double data[12];
    for (int idx = 0; idx < 2; ++idx) {
        camIfs >> camName;
        for (double &item : data) {
            camIfs >> item;
        }
        if (idx != 1)
            continue;
        K << data[0], data[1], data[2], data[4], data[5], data[6], data[8], data[9], data[10];
        t << data[3], data[7], data[11];
    }
    t = K.inverse() * t;
    K.block<2, 3>(0, 0) = 0.5 * K.block<2, 3>(0, 0);
    double baseline = t.norm();
    Camera::Ptr camera(new Camera(baseline, K));
    return camera;
}

NAMESPACE_END