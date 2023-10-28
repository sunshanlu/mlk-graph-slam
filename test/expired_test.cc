// 测试智能指针weak_ptr和lock() == nullptr是否等价
// 测试结果：等价

#include <iostream>
#include <memory>

int main() {
    std::weak_ptr<int> weak;
    // {
    std::shared_ptr<int> shared = std::make_shared<int>(100);
    weak = shared;
    // }
    std::cout << weak.expired() << std::endl;
    std::cout << (weak.lock() == nullptr) << std::endl;

    return 0;
}
