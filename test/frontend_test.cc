#include "frontend.h"

using namespace myslam;
int main() {
    Dataset::Ptr dataset = KITTISet::getInstance();
    Camera::Ptr camera = dataset->createCamera();
    Map::Ptr map = Map::getInstance();
    Frontend::Ptr frontend = Frontend::getInstance();

    frontend->setCamera(camera);
    frontend->setMap(map);
    frontend->setDataset(dataset);

    frontend->run();
    return 0;
}