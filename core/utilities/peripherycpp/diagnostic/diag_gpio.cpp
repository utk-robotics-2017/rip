#include <peripherycpp/gpio.hpp>
#include <cstdio>

using Gpio = rip::peripherycpp::Gpio

int main(int argc, char **argv)
{
    Gpio g;
    g.open(2, 0);
    g.write(1);
    g.close();
    g.open(2, 1);
    bool tmp = g.read();
    g.close();
    fprintf(stdout, "read produced %b\n", tmp);
    
}
