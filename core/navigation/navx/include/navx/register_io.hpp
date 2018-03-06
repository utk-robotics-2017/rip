/*
 * RegisterIO.h
 *
 *  Created on: Jul 29, 2015
 *      Author: Scott
 */

#ifndef SRC_REGISTERIO_H_
#define SRC_REGISTERIO_H_

#include <stdint.h>
#include "iio_provider.hpp"
#include "i_register_io.hpp"
#include "imu_protocol.hpp"
#include "navx_protocol.hpp"
#include "i_board_capabilities.hpp"
#include "iio_complete_notification.hpp"
#include <ctime>

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class RegisterIO : public IIOProvider
            {
            private:
                IRegisterIO *io_provider;
                uint8_t update_rate_hz;
                IMUProtocol::GyroUpdate raw_data_update;
                NavXProtocol::NavXUpdate navx_update;
                NavXProtocol::NavXPosUpdate navxpos_update;
                IIOCompleteNotification *notify_sink;
                IIOCompleteNotification::BoardState board_state;
                NavXProtocol::BoardID board_id;
                IBoardCapabilities *board_capabilities;
                double last_update_time;
                int byte_count;
                bool stop;
                int update_count;
                long last_sensor_timestamp;
            public:
                RegisterIO(IRegisterIO *io_provider,
                            uint8_t update_rate_hz,
                            IIOCompleteNotification *notify_sink,
                            IBoardCapabilities *board_capabilities);
                bool   isConnected();
                double getByteCount();
                double getUpdateCount();
                void   getUpdateRateHz(uint8_t update_rate);
                void   zeroYaw();
                void   zeroDisplacement();
                void   run();
                //void   stop();
                void   enableLogging(bool enable);
                virtual ~RegisterIO();
            private:
                bool   getConfiguration();
                void   getCurrentData();
            };
        }
    }
}

#endif /* SRC_REGISTERIO_H_ */
