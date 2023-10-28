#include "app.h"

NAMESPACE_BEGIN
/**
 * @brief   对前端，后端和展示类进行初始化
 *
 */
Application::Application() {
    Config::loadConfig();

    m_dataset = KITTISet::getInstance();
    m_camera = m_dataset->createCamera();
    m_map = Map::getInstance();
    m_frontend = Frontend::getInstance();
    m_backend = Backend::getInstance();
    m_viewer = Viewer::getInstance();

    m_frontend->setCamera(m_camera);
    m_frontend->setMap(m_map);
    m_frontend->setDataset(m_dataset);
    m_frontend->setViewer(m_viewer);

    m_backend->setCamera(m_camera);
    m_backend->setMap(m_map);
    m_backend->setViewer(m_viewer);

    m_viewer->setCamera(m_camera);
    m_viewer->setMap(m_map);
}

/**
 * @brief   开启线程
 *
 */
void Application::run() {
    std::thread frontendTh([this]() { m_frontend->run(); });
    std::thread backendTh([this]() { m_backend->run(); });
    std::thread viewerTh([this]() { m_viewer->run(); });

    frontendTh.join();
    backendTh.join();
    viewerTh.join();
}

NAMESPACE_END