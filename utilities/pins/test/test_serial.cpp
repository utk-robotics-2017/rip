#include <iostream>
#include <gtest/gtest.h>

#include "serial.hpp"

namespace rip
{
    namespace utilities
    {
        namespace rip
        {
            namespace serial
            {
                TEST(Serial_open, bad_file)
                {
                    ASSERT_THROW(std::make_shared<Serial>("Butts"), OpenError);
                }
            }
        }
    }
}
