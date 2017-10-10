namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class YPRUpdate
            {
            public:
                float yaw;
                float pitch;
                float roll;
                float compass_heading;

                // Yaw/Pitch/Roll (YPR) Update Packet - e.g., !y[yaw][pitch][roll][compass]
                // (All values as floats)
                struct PacketConstants
                {
                    static constexpr char kMsgId = 'y';
                    static constexpr uint8_t kYaw = 2;
                    static constexpr uint8_t kRoll = 9;
                    static constexpr uint8_t kPitch = 16;
                    static constexpr uint8_t kCompass = 23;
                    static constexpr uint8_t kChecksum = 30;
                    static constexpr uint8_t kTerminator = 32;
                    static constexpr uint8_t kMessageLength = 34;
                };
            };
        }
    }
}