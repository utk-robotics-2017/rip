#include "peripherycpp/exceptions.hpp"
#include "peripherycpp/gpio.hpp"

#include <gtest/gtest.h>
#include <googletest_rip_macros.hpp>

using Gpio = rip::peripherycpp::Gpio;
using ArgError = rip::peripherycpp::GpioArgError;
using ExportError = rip::peripherycpp::GpioExportError;
using OpenError = rip::peripherycpp::GpioOpenError;
using IoError = rip::peripherycpp::GpioIoError;
using CloseError = rip::peripherycpp::GpioCloseError;
using SetDirError = rip::peripherycpp::GpioSetDirectionError;
using GetDirError = rip::peripherycpp::GpioGetDirectionError;
using SetEdgeError = rip::peripherycpp::GpioSetEdgeError;
using GetEdgeError = rip::peripherycpp::GpioGetEdgeError;

namespace rip
{

    namespace peripherycpp
    {

        namespace test
        {
            TEST(Gpio_open, bad_direction)
            {
                Gpio g;
                unsigned int pin = 50;
                ASSERT_THROW(g.open(pin, 5), ArgError);
            }

            TEST(Gpio_open, 
        }

    }

}
