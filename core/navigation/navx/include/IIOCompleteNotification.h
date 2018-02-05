

#include <stdint.h>
#ifndef SRC_IIOCOMPLETENOTIFICATION_H_
#define SRC_IIOCOMPLETENOTIFICATION_H_

#include "IMUProtocol.h"
#include "AHRSProtocol.h"
namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class IIOCompleteNotification
            {
            public:
                IIOCompleteNotification() {}
                struct BoardState
                {
                    uint8_t op_status;
                    int16_t sensor_status;
                    uint8_t cal_status;
                    uint8_t selftest_status;
                    int16_t capability_flags;
                    uint8_t update_rate_hz;
                    int16_t accel_fsr_g;
                    int16_t gyro_fsr_dps;
                };
                virtual void setYawPitchRoll(IMUProtocol::YPRUpdate& ypr_update, long sensor_timestamp) = 0;
                virtual void setAHRSData(AHRSProtocol::AHRSUpdate& ahrs_update, long sensor_timestamp) = 0;
                virtual void setAHRSPosData(AHRSProtocol::AHRSPosUpdate& ahrs_update, long sensor_timestamp) = 0;
                virtual void setRawData(IMUProtocol::GyroUpdate& raw_data_update, long sensor_timestamp) = 0;
                virtual void setBoardID(AHRSProtocol::BoardID& board_id) = 0;
                virtual void setBoardState(BoardState& board_state) = 0;
            };
        }
    }
}

#endif /* SRC_IIOCOMPLETENOTIFICATION_H_ */
