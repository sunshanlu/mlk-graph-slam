#include "g2o_types.h"

using namespace myslam;

int main() {
    auto *vertex = new PointVertex;
    vertex->setId(100);
    std::cout << vertex->id() << std::endl;
    delete vertex;
}