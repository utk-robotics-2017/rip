#include <peripherycpp/gpio.hpp>
#include <cstdio>

using Gpio = rip::peripherycpp::Gpio;

int main(int argc, char **argv)
{
    Gpio g;
    g.open(2, 0);
    g.write(1);
    g.close();
    g.open(2, 1);
    bool tmp = g.read();
    unsigned int pnum = g.pin();
    int psuc = g.poll(50);
    bool support = g.supportsInterrupts();
    int dir = g.getDirection();
    int edge = g.getEdge();
    g.close();
    fprintf(stdout, "read produced %b\n", tmp);
    fprintf(stdout, "pin number = %i\n", pnum);
    fprintf(stdout, "Poll Successful?: %i\n", psuc);
    fprintf(stdout, "Supports Interrupts?: %i\n", support);
    fprintf(stdout, "Direction code: %i\n", dir);
    fprintf(stdout, "Edge code: %i\n", edge);
    return 0;
}
