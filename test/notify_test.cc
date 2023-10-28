#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

std::mutex mtx;
std::condition_variable cv;
bool ready = false;

// 不管怎么样，生产者线程在执行完之后，等待几微秒，让条件变量有拿锁的时间
void printHello() {
    while (1) {
        {
            std::unique_lock<std::mutex> Lck(mtx);
            std::cout << "hello" << std::endl;
            ready = true;
            cv.notify_one();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void printWorld() {
    while (1) {
        std::unique_lock<std::mutex> Lck(mtx);
        cv.wait(Lck, [] { return ready; });
        std::cout << "world" << std::endl;
        ready = false;
    }
}

int main() {
    std::thread t1(printHello);
    std::thread t2(printWorld);
    t1.join();
    t2.join();
}