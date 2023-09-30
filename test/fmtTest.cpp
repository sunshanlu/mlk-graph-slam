#include <fmt/format.h>


int main()
{
    std::size_t num = 100;
    fmt::print("{:06}", num);

    return 0;
}