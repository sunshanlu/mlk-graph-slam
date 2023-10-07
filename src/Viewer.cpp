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
            drawCurrFrame(vis_camera);
            if (m_map) drawActiveMap();
            pangolin::FinishFrame();
            cv::waitKey(1);
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
        glColor3f(m_pointColor[0], m_pointColor[1], m_pointColor[2]);
        glVertex3d(mapPoint.x(), mapPoint.y(), mapPoint.z());
    }


    Eigen::Matrix4d Viewer::drawFrame(Frame::Ptr &frame, bool isKf) const
    {
        Sophus::SE3d Twc = frame->pose().inverse();
        glPushMatrix();
        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat *) m.data());
        if (isKf) glColor3f(m_kfColor[0], m_kfColor[1], m_kfColor[2]);
        else glColor3f(m_frameColor[0], m_frameColor[1], m_frameColor[2]);
        glLineWidth(m_line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);
        glVertex3f(0, 0, 0);
        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);
        glVertex3f(0, 0, 0);
        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);
        glVertex3f(0, 0, 0);
        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);

        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);
        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);

        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);
        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);

        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);
        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);

        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);
        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);

        glEnd();
        glPopMatrix();

        return Twc.matrix();
    }

//    void Viewer::drawFrame(pangolin::OpenGlRenderState &vis_camera)
//    {
//        Sophus::SE3d Twc = m_frame->pose().inverse();
//        glPushMatrix();
//        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
//        glMultMatrixf((GLfloat *) m.data());
//        glColor3f(1, 0, 0);
//        glLineWidth(m_line_width);
//        glBegin(GL_LINES);
//        glVertex3f(0, 0, 0);
//        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);
//        glVertex3f(0, 0, 0);
//        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);
//        glVertex3f(0, 0, 0);
//        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);
//        glVertex3f(0, 0, 0);
//        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);
//
//        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);
//        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);
//
//        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);
//        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);
//
//        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (m_height - 1 - m_cy) / m_fy, m_sz);
//        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);
//
//        glVertex3f(m_sz * (0 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);
//        glVertex3f(m_sz * (m_width - 1 - m_cx) / m_fx, m_sz * (0 - m_cy) / m_fy, m_sz);
//
//        glEnd();
//        glPopMatrix();
//
//        pangolin::OpenGlMatrix matrix(Twc.matrix());
//        vis_camera.Follow(matrix, true);
//    }

    void Viewer::drawActiveMap()
    {
        for (auto &keyFrame: m_map->getActiveFrames()) {
            drawFrame(keyFrame, true);
        }
        for (auto &point: m_map->getActivePoints()) {
            drawPoint(point->position());
        }
    }
} // myslam