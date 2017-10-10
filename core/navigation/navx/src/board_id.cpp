#include "board_id.hpp"

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            BoardId::BoardId(const std::string& buffer)
            {

            }

            std::string BoardId::encode()
            {
                std::string buffer;
                buffer.resize(BoardIdentityResponsePacket::kMessageLength);

                // Header
                buffer[0] =
                    buffer[1] =
                        buffer[2] = BoardIdentityResponsePacket::kMessageLength - 2;
                buffer[3] = BoardIdentityResponsePacket::kMsgId;

                // Data
                buffer[BoardIdentityResponsePacket::kBoardType] = type;
                buffer[BoardIdentityResponsePacket::kHardwareRev] = hw_rev;
                buffer[BoardIdentityResponsePacket::kFirmwareMajor] = fw_major;
                buffer[BoardIdentityResponsePacket::kFirmwareMinor] = fw_minor;

                for (int i = 0; i < 12; i++)
                {
                    buffer[BoardIdentityResponsePacket::kUniqueId0 + i] = unique_id[i];
                }


            }
        }
    }
}