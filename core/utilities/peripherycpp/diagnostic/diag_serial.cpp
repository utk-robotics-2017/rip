#include <peripherycpp/serial.hpp>
#include <cstdio>

using Serial = rip::peripherycpp::Serial

int main(int arc, char **argv)
{
    Serial s;
    s.open("/dev/ttyAMA0", 115200, 8, 0, 1, 1, 1);
    std::string str = "Hi";
    std::vector<uint8_t> data(str.begin(), str.end());
    s.write(data);
    unsigned int db = s.getDatabits();
    uint32_t baud = s.getBaudrate();
    int par = s.getParity();
    unsigned int sb = s.getStopbits();
    bool x = s.getxOnxOff();
    bool r = s.getRtscts();
    std::vector<uint8_t> tmp = s.read(2, 100);
    std::string rd(tmp.begin(), tmp.end());
    s.close();
    fprintf(stdout, "read = %s\ndatabits = %i\nbaudrate = %lu\nparity = %i\nstopbits = %i\nSoftware Control = %b\nHardware Control = %b\n", rd.c_str(), db, baud, par, sb, x, r);
    return 0;
}
