#include "peripherycpp/exceptions.hpp"
#include "peripherycpp/gpio.hpp"
#include "peripherycpp/mock/mock_gpio.hpp"

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <googletest_rip_macros.hpp>

using MockGpio = rip::peripherycpp::mock::MockGpio
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
                MockGpio mock;
                EXPECT_CALL(mock, open(5, 5))
                    .WillOnce(Throw(ArgError));
                unsigned int pin = 5;
                Gpio g(&mock);
                ASSERT_THROW(g.open(pin, 5), ArgError);
            }

            TEST(Gpio_open, bad_pinnum)
            {
                MockGpio mock;
                EXPECT_CALL(mock, open(-1, 0))
                    .WillOnce(Throw(ExportError));
                int pin = -1;
                Gpio g(&mock);
                ASSERT_THROW(g.open(pin, 0), ExportError); 
            }

        }

    }

}
