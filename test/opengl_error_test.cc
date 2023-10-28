#include <pangolin/pangolin.h>

#include "dataset.h"

using namespace myslam;

void drawFrame(pangolin::GlTexture &texture, Frame::Ptr &frame) {
    cv::Mat dst;
    cv::cvtColor(frame->m_leftIMG, dst, cv::COLOR_GRAY2BGR);
    // for (auto &feature : m_frame->getLeftFeatures()) {
    //     if (feature->m_mapPoint.lock()) {
    //         double xval = feature->m_position.x();
    //         double yval = feature->m_position.y();
    //         cv::circle(dst, cv::Point((int)xval, (int)yval), 2, FEATURE_COLOR, 2);
    //     }
    // }
    cv::flip(dst, dst, 0);
    texture.Upload(dst.datastart, GL_BGR, GL_UNSIGNED_BYTE);
    texture.RenderToViewport();
}

int main() {
    Dataset::Ptr dataset = KITTISet::getInstance();
    const int width = 1241;
    const int height = 376;

    pangolin::CreateWindowAndBind("MLK-SLAM", width, height);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::View &image =
        pangolin::Display("IMG").SetBounds(0.0, 1.0, 0.0, 1.0).SetAspect(-width / (float)height);
    pangolin::GlTexture texture(width / 2, height / 2, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
    while (!pangolin::ShouldQuit()) {
        Frame::Ptr frame = dataset->createFrame();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        image.Activate();
        glColor3f(1.0f, 1.0f, 1.0f);
        drawFrame(texture, frame);
        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    dataset->createFrame();
}