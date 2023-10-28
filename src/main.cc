#include "app.h"

using namespace myslam;
int main() {
    Application::Ptr app = Application::getInstance();
    app->run();
    return 0;
}