#include "camera.h"
#include "dataset.h"
#include "map.h"
#include <opencv2/opencv.hpp>

using namespace myslam;

int main() {
    Dataset::Ptr dataset = KITTISet::getInstance();
    Camera::Ptr camera = dataset->createCamera();
    Frame::Ptr frame = dataset->createFrame();

    std::cout << camera->m_K << std::endl;
    std::cout << camera->m_baseline << std::endl;
    cv::imshow("leftIMG", frame->m_leftIMG);
    cv::imshow("rightIMG", frame->m_rightIMG);
    std::cout << frame->m_id << std::endl;
    cv::waitKey(0);

    frame = dataset->createFrame();
    cv::imshow("leftIMG", frame->m_leftIMG);
    cv::imshow("rightIMG", frame->m_rightIMG);
    std::cout << frame->m_id << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}