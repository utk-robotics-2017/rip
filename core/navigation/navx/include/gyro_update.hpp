namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class GyroUpdate
            {
            public:
                int16_t gyro_x;
                int16_t gyro_y;
                int16_t gyro_z;
                int16_t accel_x;
                int16_t accel_y;
                int16_t accel_z;
                int16_t mag_x;
                int16_t mag_y;
                int16_t mag_z;
                float   temp_c;

                // Gyro/Raw Data Update packet - e.g., !g[gx][gy][gz][accelx][accely][accelz][magx][magy][magz][temp_c]
                struct PacketConstants
                {
                    static constexpr char kMsgId            = 'g';
                    static constexpr uint8_t kGyroX         = 2;
                    static constexpr uint8_t kGyroY         = 6;
                    static constexpr uint8_t kGyroZ         = 10;
                    static constexpr uint8_t kAccelX        = 14;
                    static constexpr uint8_t kAccelY        = 18;
                    static constexpr uint8_t kAccelZ        = 22;
                    static constexpr uint8_t kMagX          = 26;
                    static constexpr uint8_t kMagY          = 30;
                    static constexpr uint8_t kMagZ          = 34;
                    static constexpr uint8_t kTemp          = 38;
                    static constexpr uint8_t kChecksum      = 42;
                    static constexpr uint8_t kTerminator    = 44;
                    static constexpr uint8_t kMessageLength = 46;
                };
            };
        } // navx
    } // navigation
} // rip