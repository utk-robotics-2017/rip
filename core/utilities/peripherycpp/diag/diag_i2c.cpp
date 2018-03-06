#include <peripherycpp/i2c.hpp>

using I2c = rip::peripherycpp::I2c;

int main(int argc, char** argv)
{
    I2c i;
    i.open("/dev/i2c-0");
    std::string tmp1 = "Hi";
    std::string tmp2 = "Y'all";
    std::vector<uint8_t> v1(tmp1.begin(), tmp1.end());
    std::vector<uint8_t> v2(tmp2.begin(), tmp2.end());
    std::vector< std::vector<uint8_t> > v;
    v.push_back(v1);
    v.push_back(v2);
    std::vector<int> f;
    f.push_back(0);
    f.push_back(1);
    i.transfer(v, f, 2);
    i.close();
    return 0;
}
