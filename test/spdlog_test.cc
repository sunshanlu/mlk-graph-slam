#include "spdlog/spdlog.h"
#include <iostream>

int main() {
    spdlog::set_pattern("[%H:%M:%S %s:%#][%^--%L--%$] %v");

    SPDLOG_INFO({}, "Hello World");
    return 0;
}
