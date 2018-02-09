#ifndef _PERIPHERY_I2C_HPP
#define _PERIPHERY_I2C_HPP

extern "C"
{
    #include "i2c.h"
}
#include <vector>
#include <string>

namespace rip
{

    namespace periphery
    {

        namespace i2c
        {

            class i2c 
            {
                public:

                    void open(const std::string path);

                    void transfer(std::vector< vector<uint8_t> > msg_data, std::vector<int> flags, size_t count);

                    void close();

                    int fd();

                    std::string tostring(size_t len);

                private:

                    i2c_t i2c;

            };

        }

    }

}

#endif
