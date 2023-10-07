#include <thread>

#include "myslam/Camera.h"
#include "myslam/Dataset.h"
#include "myslam/Map.h"
#include "myslam/Viewer.h"
#include "myslam/FrontEnd.h"
#include "myslam/BackEnd.h"

int main()
{
    myslam::Dataset::Ptr dataset = std::make_shared<myslam::Dataset>
            ("/media/lu/Ventoy/Dataset/00");
    myslam::Camera::Ptr camera = dataset->createCamera();
    myslam::FrontEnd::Ptr frontEnd = myslam::FrontEnd::getInstance();
    myslam::Viewer::Ptr viewer = myslam::Viewer::getInstance();
    myslam::Map::Ptr map = myslam::Map::getInstance();
    myslam::BackEnd::Ptr backend = myslam::BackEnd::getInstance();

    frontEnd->setDataset(dataset);
    frontEnd->setCamera(camera);
    frontEnd->setViewer(viewer);
    frontEnd->setMap(map);

    backend->setCamera(camera);
    backend->setMap(map);

    viewer->setMap(map);

    std::function<void(myslam::FrontEnd::Ptr)> frontEndFunc = [](const myslam::FrontEnd::Ptr &frontend) {
        frontend->run();
    };
    std::function<void(myslam::Viewer::Ptr)> viewerFunc = [](const myslam::Viewer::Ptr &viewer) {
        viewer->run();
    };

    std::function<void(myslam::BackEnd::Ptr)> backEndFunc = [](const myslam::BackEnd::Ptr &backEnd) {
        backEnd->run();
    };

    std::thread frontEndThread(frontEndFunc, frontEnd);
    std::thread viewerThread(viewerFunc, viewer);
    std::thread backendThread(backEndFunc, backend);

    frontEndThread.join();
    viewerThread.join();
    backendThread.join();

    return 0;
}