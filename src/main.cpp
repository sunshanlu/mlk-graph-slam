#include <thread>

#include "myslam/Camera.h"
#include "myslam/Dataset.h"
#include "myslam/Map.h"
#include "myslam/Viewer.h"
#include "myslam/FrontEnd.h"

int main()
{
    myslam::Dataset::Ptr dataset = std::make_shared<myslam::Dataset>
            ("/media/rookie-lu/DATA1/Dataset/KITTI-ODOM/00");
    myslam::Camera::Ptr camera = dataset->createCamera();
    myslam::FrontEnd::Ptr frontEnd = myslam::FrontEnd::getInstance();
    myslam::Viewer::Ptr viewer = myslam::Viewer::getInstance();
    myslam::Map::Ptr map = myslam::Map::getInstance();

    frontEnd->setDataset(dataset);
    frontEnd->setCamera(camera);
    frontEnd->setViewer(viewer);
    frontEnd->setMap(map);
//    frontEnd->run();

    std::function<void(myslam::FrontEnd::Ptr)> frontEndFunc = [](const myslam::FrontEnd::Ptr &frontend) {
        frontend->run();
    };
    std::function<void(myslam::Viewer::Ptr)> viewerFunc = [](const myslam::Viewer::Ptr &viewer) {
        viewer->run();
    };

    std::thread frontEndThread(frontEndFunc, frontEnd);
    std::thread viewerThread(viewerFunc, viewer);

    frontEndThread.join();
    viewerThread.join();

    return 0;
}