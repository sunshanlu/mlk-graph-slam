#include "viewer.h"

NAMESPACE_BEGIN

void Viewer::run() {
    const int width = 1241;
    const int height = 752;
    const double fx = m_camera->m_K(0, 0) * 2;
    const double fy = m_camera->m_K(1, 1) * 2;
    const double cx = m_camera->m_K(0, 2) * 2;
    const double cy = m_camera->m_K(1, 2) * 2;
    pangolin::CreateWindowAndBind("MLK-SLAM", width, height);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState renderState(
        pangolin::ProjectionMatrix(width, height / 2, fx, fy, cx, cy, 0.01, 200),
        pangolin::ModelViewLookAt(0, -10, -30, 0, 0, 0, 0.0, -1.0, 0.0));
    pangolin::Handler3D handler(renderState);

    pangolin::View &map =
        pangolin::Display("Map").SetAspect(-(float)width / (float)height * 2).SetHandler(&handler);

    pangolin::GlTexture texture(width / 2, height / 4, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
    pangolin::View &image = pangolin::Display("IMG");

    pangolin::View &multi = pangolin::Display("Multi")
                                .SetBounds(0.0, 1.0, 0.0, 1.0)
                                .SetLayout(pangolin::LayoutEqualVertical)
                                .AddDisplay(map)
                                .AddDisplay(image);

    while (!pangolin::ShouldQuit() && !m_isStop) {
        if (m_frame == nullptr) {
            continue;
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        map.Activate(renderState);
        drawCurrFrame();
        followCurrFrame(renderState);
        drawKeyFrames();
        drawMapPoints();

        image.Activate();
        glColor3f(1.0f, 1.0f, 1.0f);
        drawImage(texture);
        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    setFlag();
}

/**
 * @brief   在Pangolin中绘制Frame
 * @param   [in]frame: 指定Frame
 * @param   [in]color: 指定颜色信息
 *
 * @details 根据Frame的外参信息，和Camera的内参信息，
 *          将(0, 0), (width, 0), (width, height)
 *          ，(0, height)和相机光心连接，组成的几何来
 *          表示frame在世界坐标系下的位姿信息
 *
 */
void Viewer::drawFrame(const Frame &frame, const float *color) {
    static const double fx = m_camera->m_K(0, 0) * 2;
    static const double fy = m_camera->m_K(1, 1) * 2;
    static const double cx = m_camera->m_K(0, 2) * 2;
    static const double cy = m_camera->m_K(1, 2) * 2;
    static const double width = m_frame->m_leftIMG.cols * 2;
    static const double height = m_frame->m_leftIMG.rows * 2;

    SE3d T_wc = frame.pose().inverse();
    Mat4f Mat_wc = T_wc.matrix().cast<float>();

    glPushMatrix();
    glMultMatrixf((GLfloat *)Mat_wc.data());

    glColor3f(color[0], color[1], color[2]);
    glLineWidth(Config::LINE_WIDTH);
    glBegin(GL_LINES);

    glVertex3f(0, 0, 0);
    glVertex3f((0 - cx) / fx, (0 - cy) / fy, 1.0);

    glVertex3f(0, 0, 0);
    glVertex3f((width - cx) / fx, (0 - cy) / fy, 1.0);

    glVertex3f(0, 0, 0);
    glVertex3f((width - cx) / fx, (height - cy) / fy, 1.0);

    glVertex3f(0, 0, 0);
    glVertex3f((0 - cx) / fx, (height - cy) / fy, 1.0);

    glVertex3f((0 - cx) / fx, (0 - cy) / fy, 1.0);
    glVertex3f((width - cx) / fx, (0 - cy) / fy, 1.0);

    glVertex3f((width - cx) / fx, (0 - cy) / fy, 1.0);
    glVertex3f((width - cx) / fx, (height - cy) / fy, 1.0);

    glVertex3f((width - cx) / fx, (height - cy) / fy, 1.0);
    glVertex3f((0 - cx) / fx, (height - cy) / fy, 1.0);

    glVertex3f((0 - cx) / fx, (height - cy) / fy, 1.0);
    glVertex3f((0 - cx) / fx, (0 - cy) / fy, 1.0);

    glEnd();
    glPopMatrix();
}

/**
 * @brief   绘制含有三维信息的特征点
 * @param   [in]texture: 用来保存，更新和渲染图片信息
 *
 */
void Viewer::drawImage(pangolin::GlTexture &texture) {
    std::unique_lock<std::mutex> lck(m_mutex);
    cv::Mat dst;
    cv::cvtColor(m_frame->m_leftIMG, dst, cv::COLOR_GRAY2BGR);
    for (auto &feature : m_frame->getLeftFeatures()) {
        if (feature->m_mapPoint.lock()) {
            double xval = feature->m_position.x();
            double yval = feature->m_position.y();
            cv::circle(dst, cv::Point((int)xval, (int)yval), 2, Config::FEATURE_COLOR, 2);
        }
    }
    cv::flip(dst, dst, 0);
    texture.Upload(dst.datastart, GL_BGR, GL_UNSIGNED_BYTE);
    texture.RenderToViewport();
}

/**
 * @brief 绘制地图中激活的地图点和激活的关键帧
 *
 */
void Viewer::drawKeyFrames() {
    std::vector<Frame::Ptr> activeFrames;
    {
        std::unique_lock<std::mutex> mapLck(m_map->m_mutex);
        for (auto &frame : m_map->getActiveFrames())
            activeFrames.push_back(frame.second);
    }
    for (auto &frame : activeFrames)
        drawFrame(*frame, Config::MAP_FRAME_COLOR);
}

/**
 * @brief   在pangolin窗口中，绘制激活的地图点
 * @param   [in]activePoints: 激活的地图点
 * @note    不存在GL_POINT这个东西，绘制单个点也需要使用GL_POINTS
 *
 */
void Viewer::drawMapPoints() {
    glPointSize(Config::MAPPOINT_SIZE);
    glBegin(GL_POINTS);
    for (const auto &point : m_points) {
        glColor3f(Config::MAPPOINT_COLOR[0], Config::MAPPOINT_COLOR[1], Config::MAPPOINT_COLOR[2]);
        glVertex3f(point->m_position.x(), point->m_position.y(), point->m_position.z());
    }
    glEnd();
}

void Viewer::followCurrFrame(pangolin::OpenGlRenderState &renderState) {
    Mat4d Twc = m_frame->pose().inverse().matrix();
    pangolin::OpenGlMatrix m(Twc);
    renderState.Follow(m, true);
}
NAMESPACE_END
