#include "integration_control.hpp"

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            IntegrationControl::IntegrationControl()
            {
                if ((buffer[0] != kPacketStartChar) ||
                        (buffer[1] != kBinaryPacketIndicatorChar) ||
                        (buffer[2] != IntegrationControlCommand::kMessageLength - 2 && buffer[2] != IntegrationControlResponse::kMessageLength - 2) ||
                        (buffer[3] != IntegrationControlCommand::kMsgId && buffer[3] != IntegrationControlResponse::kMsgId))
                {
                    // todo(Andrew): throw exception
                }

                if (buffer[3] == IntegrationControlCommand::kMsgId)
                {
                    if (!verifyChecksum(buffer, IntegrationControlCommand::kChecksumIndex))
                    {
                        // todo(Andrew): throw exception
                    }

                    // Data
                    action = static_cast<uint8_t>(buffer[IntegrationControlCommand::kActionIndex]);
                    parameter = ImuRegisters::decodeProtocolInt32(&buffer[IntegrationControlCommand::kParameterIndex]);
                }
                else if (buffer[3] == IntegrationControlResponse::kMsgId)
                {
                    if (!verifyChecksum(buffer, IntegrationControlResponse::kChecksumIndex))
                    {
                        // todo(Andrew): throw exception
                    }

                    // Data
                    action = static_cast<uint8_t>(buffer[IntegrationControlResponse::kActionIndex]);
                    parameter = ImuRegisters::decodeProtocolInt32(&buffer[IntegrationControlResponse::kParameterIndex]);
                }
                // else has already been checked by first if statement
            }

            std::string IntegrationControl::encode()
            {
                std::string buffer;
                buffer.resize(IntegrationControlCommand::kMessageLength);

                // Header
                buffer[0] = kPacketStartChar;
                buffer[1] = kBinaryPacketIndicatorChar;
                buffer[2] = IntegrationControlCommand::kMessageLength - 2;
                buffer[3] = IntegrationControlCommand::kMsgId;

                // Data
                buffer[IntegrationControlCommand::kActionIndex] = action;
                ImuRegisters::encoderProtocolInt32(parameter, &buffer[IntegrationControlCommand::kParameterIndex]);

                // Footer
                AhrsProtocol::encodeTermination(buffer, IntegrationControlCommand::kMessageLength, IntegrationControlCommand::kMessageLength - 4);

                return buffer;
            }

            std::string IntegrationControl::encodeResponse()
            {
                std::string buffer;
                buffer.resize(IntegrationControlResponse::kMessageLength);

                // Header
                buffer[0] = kPacketStartChar;
                buffer[1] = kBinaryPacketIndicatorChar;
                buffer[2] = IntegrationControlResponse::kMessageLength - 2;
                buffer[3] = IntegrationControlResponse::kMsgId;

                // Data
                buffer[IntegrationControlResponse::kActionIndex] = action;
                ImuRegisters::encoderProtocolInt32(parameter, &buffer[IntegrationControlResponse::kParameterIndex]);

                // Footer
                AhrsProtocol::encodeTermination(buffer, IntegrationControlResponse::kMessageLength, IntegrationControlResponse::kMessageLength - 4);

                return buffer;
            }
        } // navx
    } // navigation
} // rip