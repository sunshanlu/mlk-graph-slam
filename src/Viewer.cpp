//
// Created by rookie-lu on 23-9-27.
//
#include <chrono>

#include <pangolin/pangolin.h>

#include "myslam/Viewer.h"

namespace myslam
{
    Viewer::Ptr Viewer::getInstance()
    {
        static Ptr viewer(new Viewer);
        return viewer;
    }

    void Viewer::run()
    {
        pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
                pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));
        pangolin::View &vis_display = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(vis_camera));

        while (true) {
            if (pangolin::ShouldQuit()) {
                cv::destroyAllWindows();
                break;
            }
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            vis_display.Activate(vis_camera);
            if (!m_frame) continue;
            cv::Mat img;
            drawIMG(img);
            cv::imshow("currLeftIMG", img);
            cv::waitKey(1);
            drawFrame(vis_camera);

            pangolin::FinishFrame();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // sleep for 5ms
        }
    }

    void Viewer::drawIMG(cv::Mat &img)
    {

        cv::cvtColor(m_frame->getLeftIMG(), img, CV_GRAY2BGR);
        glPointSize(2);
        glBegin(GL_POINTS);
        for (const auto &feature: m_frame->getLeftFeatures()) {
            if (feature == nullptr || feature->isOutlier()) continue;
            cv::circle(img, cv::Point((int) feature->position()[0], (int) feature->position()[1]), 2,
                       cv::Scalar(0, 250, 0), 2);
            // auto mapPoint = feature->mapPoint().lock();
            // drawPoint(mapPoint->position());
        }
        glEnd();
    }

    void Viewer::drawPoint(const Eigen::Vector3d &mapPoint)
    {
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3d(mapPoint.x(), mapPoint.y(), mapPoint.z());
    }


    void Viewer::drawFrame(pangolin::OpenGlRenderState &vis_camera)
    {
        Sophus::SE3d Twc = m_frame->pose().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;
        glPushMatrix();
        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat *) m.data());
        glColor3f(1, 0, 0);
        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();

        pangolin::OpenGlMatrix matrix(Twc.matrix());
        vis_camera.Follow(matrix, true);
    }
} // myslam