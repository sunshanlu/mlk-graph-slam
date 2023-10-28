#include "config.h"
#include "frontend.h"
#include "viewer.h"

using namespace myslam;
int main() {
    Config::loadConfig();

    Dataset::Ptr dataset = KITTISet::getInstance();
    Camera::Ptr camera = dataset->createCamera();
    Map::Ptr map = Map::getInstance();
    Frontend::Ptr frontend = Frontend::getInstance();
    Viewer::Ptr viewer = Viewer::getInstance();

    frontend->setCamera(camera);
    frontend->setMap(map);
    frontend->setDataset(dataset);
    frontend->setViewer(viewer);

    frontend->run();
    return 0;
}