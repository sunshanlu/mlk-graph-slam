#include "frontend.h"
#include <thread>

using namespace myslam;
int main() {
    Dataset::Ptr dataset = KITTISet::getInstance();
    Camera::Ptr camera = dataset->createCamera();
    Map::Ptr map = Map::getInstance();
    Frontend::Ptr frontend = Frontend::getInstance();
    Viewer::Ptr viewer = Viewer::getInstance();

    viewer->setCamera(camera);
    viewer->setMap(map);
    frontend->setCamera(camera);
    frontend->setMap(map);
    frontend->setDataset(dataset);
    frontend->setViewer(viewer);

    std::thread frontendT([&frontend]() { frontend->run(); });
    std::thread viewerT([&viewer]() { viewer->run(); });
    frontendT.join();
    viewerT.join();
    return 0;
}