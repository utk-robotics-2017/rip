#include <peripherycpp/mmio.hpp>

using Mmio = rip::peripherycpp::Mmio;

int main(int argc, char** argv)
{
    Mmio m;
    m.open(0x3F000000, 8);
    std::string stmp = "Hi";
    std::vector<uint8_t> data(stmp.begin(), stmp.end());
    m.write(0, data);
    std::vector<uint8_t> vtmp = m.read(0, 2);
    std::string str(vtmp.begin(), vtmp.end());
    uintptr_t b = m.base();
    int s = m.size();
    m.close();
    printf("read = %s\nbase = %p\nsize = %i\n", str.c_str(), b, s);
    return 0;
}
