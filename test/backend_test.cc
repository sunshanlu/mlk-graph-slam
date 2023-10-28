#include <thread>

#include "backend.h"
#include "frontend.h"
#include "viewer.h"
#include "config.h"

using namespace myslam;
int main() {
    Config::loadConfig();
    Dataset::Ptr dataset = KITTISet::getInstance();
    Camera::Ptr camera = dataset->createCamera();
    Map::Ptr map = Map::getInstance();
    Frontend::Ptr frontend = Frontend::getInstance();
    Backend::Ptr backend = Backend::getInstance();
    Viewer::Ptr viewer = Viewer::getInstance();

    frontend->setCamera(camera);
    frontend->setMap(map);
    frontend->setDataset(dataset);
    frontend->setViewer(viewer);

    backend->setCamera(camera);
    backend->setMap(map);
    backend->setViewer(viewer);

    viewer->setCamera(camera);
    viewer->setMap(map);

    std::thread frontendTh([frontend]() { frontend->run(); });
    std::thread backendTh([backend]() { backend->run(); });
    std::thread viewerTh([viewer]() { viewer->run(); });

    frontendTh.join();
    backendTh.join();
    viewerTh.join();
    return 0;
}