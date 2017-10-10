#ifndef BOARD_ID_HPP
#define BOARD_ID_HPP

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            /**
             *
             */
            class BoardId
            {
            public:
                uint8_t type;
                uint8_t hw_rev;
                uint8_t firmware_version_major;
                uint8_t firmware_version_minor;
                uint8_t firmware_revision;
                uint8_t unique_id[12];

                /**
                 * Default Constructor
                 */
                BoardId() = default;

                /**
                 * Constructor that decodes a message buffer
                 * @param buffer
                 */
                BoardId(const std::string& buffer);

                /**
                 * Encodes this object into a message buffer to send
                 * @returns The message buffer
                 */
                std::string encode();

            private:
                struct BoardIdentityResponsePacket
                {
                    // Sent in response to a Data Get (Board ID) message
                    static constexpr char kMsgId                       = 'i';
                    static constexpr uint8_t kBoardType                = 4;
                    static constexpr uint8_t kHardwareRev              = 5;
                    static constexpr uint8_t kFirmwareVersionMajor     = 6;
                    static constexpr uint8_t kFirmwareVersionMinor     = 7;
                    static constexpr uint8_t kFirmwareVersionRevision  = 8;
                    static constexpr uint8_t kUniqueId0                = 10;
                    static constexpr uint8_t kUniqueId1                = 11;
                    static constexpr uint8_t kUniqueId2                = 12;
                    static constexpr uint8_t kUniqueId3                = 13;
                    static constexpr uint8_t kUniqueId4                = 14;
                    static constexpr uint8_t kUniqueId5                = 15;
                    static constexpr uint8_t kUniqueId6                = 16;
                    static constexpr uint8_t kUniqueId7                = 17;
                    static constexpr uint8_t kUniqueId8                = 18;
                    static constexpr uint8_t kUniqueId9                = 19;
                    static constexpr uint8_t kUniqueId10               = 20;
                    static constexpr uint8_t kUniqueId11               = 21;
                    static constexpr uint8_t kChecksum                 = 22;
                    static constexpr uint8_t kTerminator               = 24;
                    static constexpr uint8_t kMessageLength            = 25;
                };
            };
        } // navx
    } // navigation
} // rip

#endif // BOARD_ID_HPP