//
// Created by rookie-lu on 23-9-26.
//

#include <vector>
#include <memory>
#include <iostream>

int main()
{
    std::vector<std::shared_ptr<int>> ptrVec;
    ptrVec.resize(10, nullptr);
    for (auto &ptr: ptrVec) {
        if (ptr.use_count() == 0)
            std::cout << "this is right" << std::endl;
    }
}