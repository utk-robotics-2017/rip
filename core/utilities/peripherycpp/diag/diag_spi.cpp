#include <peripherycpp/spi.hpp>

using Spi = rip::peripherycpp::Spi;

int main(int argc, char** argv)
{
    Spi s;
    s.open("/dev/spidev0.0", 0, 25000000);
    uint8_t *txbuf = new uint8_t[2];
    txbuf[0] = 'H';
    txbuf[1] = 'i';
    uint8_t *rxbuf;
    s.transfer(txbuf, rxbuf, 2);
    unsigned int mode = s.getMode();
    uint32_t speed = s.getMaxSpeed();
    int bo = s.getBitOrder();
    uint8_t bpw = s.getBitsPerWord();
    uint8_t flags = s.getExtraFlags();
    s.close();
    printf("rxbuf = %s\nmode = %i\nspeed = %lu\nbit order = %i\nbits per word = %hi\nflags = %hi\n", (char *)(rxbuf), mode, speed, bo, bpw, flags);
    return 0;
}
