
#ifndef SRC_SERIALIO_H_
#define SRC_SERIALIO_H_

#include "serial_port.hpp"
#include "iio_provider.hpp"
#include <stdint.h>
#include <string.h>
#include "navx_protocol.hpp"
#include "imu_protocol.hpp"
#include "iio_complete_notification.hpp"
#include "i_board_capabilities.hpp"

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class SerialIO : public IIOProvider
            {
                std::string serial_port_id;
                SerialPort* serial_port;
                uint8_t next_integration_control_action;
                bool signal_transmit_integration_control;
                bool signal_retransmit_stream_config;
                bool bstop;
                uint8_t update_type; //IMUProtocol.MSGID_XXX
                uint8_t update_rate_hz;
                int byte_count;
                int update_count;
                IMUProtocol::YPRUpdate ypr_update_data;
                IMUProtocol::GyroUpdate gyro_update_data;
                NavXProtocol::NavXUpdate navx_update_data;
                NavXProtocol::NavXPosUpdate navxpos_update_data;
                NavXProtocol::NavXPosTSUpdate navxpos_ts_update_data;
                NavXProtocol::BoardID board_id;
                IIOCompleteNotification* notify_sink;
                IIOCompleteNotification::BoardState board_state;
                IBoardCapabilities* board_capabilities;
                double last_valid_packet_time;

            public:
                SerialIO(std::string port_id,
                         uint8_t update_rate_hz,
                         bool processed_data,
                         IIOCompleteNotification* notify_sink,
                         IBoardCapabilities* board_capabilities);
                bool isConnected();
                double getByteCount();
                double getUpdateCount();
                void setUpdateRateHz(uint8_t update_rate);
                void zeroYaw();
                void zeroDisplacement();
                void run();
                void stop();
            private:
                SerialPort* resetSerialPort();
                SerialPort* getMaybeCreateSerialPort();
                void enqueueIntegrationControlMessage(uint8_t action);
                void dispatchStreamResponse(IMUProtocol::StreamResponse& response);
                int decodePacketHandler(char* received_data, int bytes_remaining);
            };
        }
    }
}
#endif /* SRC_SERIALIO_H_ */
