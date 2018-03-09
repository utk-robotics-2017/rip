#ifndef _PERIPHERY_SPI_HPP
#define _PERIPHERY_SPI_HPP

extern "C"
{
    #include "spi.h"
}
#include "peripherycpp/exceptions.hpp"
#include <string>
#include <misc/logger.hpp>
#include <fmt/format.h>

namespace rip
{

    namespace peripherycpp
    {

        class Spi
        {
            public:
                /**
                 * default constructor
                 */
                Spi();
                /**
                 * Constructor that opens spi interface
                 * @param path      the path for the spidev device
                 * @param mode      the SPI mode that the spidev device will be opened in
                 * @param max_speed the max speed (in Hz) for the spidev device.
                 */
                Spi(const std::string path, unsigned int mode, uint32_t max_speed);
                /**
                 * Spi constructor, openadvanced
                 * @param path  the path for the spidev device
                 * @param mode the SPI mode that the spidev device will be opened in
                 * @param max_speed  the max speed (in Hz) for the spidev device.
                 * @param bit_order  an int representing the bit order (MSB or LSB first).
                 * @param bits_per_word  the number of bits per word
                 * @param extra_flags  any additional flags
                 * @brief  Open the spidev device at the specified path, with the specified SPI mode,
                 * max speed, bit order, bits per word, and extra flags.
                 */
                Spi(const std::string path, unsigned int mode, uint32_t max_speed,
                        int bit_order, uint8_t bits_per_word, uint8_t extra_flags);
                /**
                 * open
                 * @param path  the path for the spidev device
                 * @param mode  the SPI mode that the spidev device will be opened in
                 * @param max_speed  the max speed (in Hz) for the spidev device.
                 * @brief  Open the spidev device at the specified path, with the specifed mode
                 * specified max speed (in Hz), and the defaults of MSB_FIRST bit order, and
                 * 8 bits per word.
                 */
                void open(const std::string path, unsigned int mode, uint32_t max_speed);

                /**
                 * openAdvanced
                 * @param path  the path for the spidev device
                 * @param mode the SPI mode that the spidev device will be opened in
                 * @param max_speed  the max speed (in Hz) for the spidev device.
                 * @param bit_order  an int representing the bit order (MSB or LSB first).
                 * @param bits_per_word  the number of bits per word
                 * @param extra_flags  any additional flags
                 * @brief  Open the spidev device at the specified path, with the specified SPI mode,
                 * max speed, bit order, bits per word, and extra flags.
                 */
                void openAdvanced(const std::string path, unsigned int mode, uint32_t max_speed,
                                   int bit_order, uint8_t bits_per_word, uint8_t extra_flags);

                /**
                 * transfer
                 * @param txbuf  the data that will be transferred to the spidev device
                 * @param rxbuf  the data that will be transferred from the spidev device
                 * @param len  the number of words that will transferred to/from the spidev device
                 * @brief  Shift out len word counts of the txbuf buffer, while shifting in len word
                 * counts to the rxbuf buffer.
                 */
                void transfer(const uint8_t *txbuf, uint8_t *rxbuf, size_t len);

                /**
                 * close
                 * @brief  Close the spidev device.
                 */
                void close();

                /**
                 * getMode
                 * @brief  Query the mode of the underlying spidev device.
                 * @return  the mode of the underlying spidev device
                 */
                unsigned int getMode();

                /**
                 * getMaxSpeed
                 * @brief  Query the max speed of the underlying spidev device.
                 * @return  the max speed (in Hz) of the underlying spidev device.
                 */
                uint32_t getMaxSpeed();

                /**
                 * getBitOrder
                 * @brief  Query the bit order of the underlying spidev device.
                 * @return  an int representing the bit order of the underlying spidev device.
                 */
                int getBitOrder();

                /**
                 * getBitsPerWord
                 * @brief  Query the bits per word of the underlying spidev device.
                 * @return  the bits per word of the underlying spidev device.
                 */
                uint8_t getBitsPerWord();

                /**
                 * getExtraFlags
                 * @brief  Query the extra flags of the underlying spidev device.
                 * @return  the extra flags of the underlying spidev device.
                 */
                uint8_t getExtraFlags();

                /**
                 * setMode
                 * @param mode  the mode that the spidev device will be set with
                 * @brief  Set the mode on the underlying spidev device.
                 */
                void setMode(unsigned int mode);

                /**
                 * setMaxSpeed
                 * @param max_speed  the max speed (in Hz) that the spidev device will be set with
                 * @brief  Set the max speed on the underlying spidev device.
                 */
                void setMaxSpeed(uint32_t max_speed);

                /**
                 * setBitOrder
                 * @param bit_order  the bit_order that the spidev device will be set with
                 * @brief  Set the bit_order on the underlying spidev device.
                 */
                void setBitOrder(int bit_order);

                /**
                 * setBitsPerWord
                 * @param bits_per_word  the bits per word that the spidev device will be set with
                 * @brief  Set the bits per word on the underlying spidev device.
                 */
                void setBitsPerWord(uint8_t bits_per_word);

                /**
                 * setExtraFlags
                 * @param extra_flags  the extra flags that the spidev device will be set with
                 * @brief  Set the extra flags on the underlying spidev device.
                 */
                void setExtraFlags(uint8_t extra_flags);

                /**
                 * fd
                 * @brief  Return the file descriptor (for the underlying spidev device) of the SPI handle
                 * @return  the file descriptor of the SPI handle
                 */
                int fd();

                /**
                 * toString
                 * @param len  the size of the string to be returned
                 * @brief  Return a string representation of the SPI handle.
                 * @return  the string representation of the SPI handle.
                 */
                std::string toString(size_t len);

            private:
                /**
                 * checkError
                 * @param err_code An int error code from Periphery Serial
                 * @brief Acts as a error handler for the Serial class
                 */
                void checkError(int err_code);

                spi_t m_spi;
        };

    }

}

#endif
