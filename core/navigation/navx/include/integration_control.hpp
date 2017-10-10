#ifndef INTEGRATION_CONTROL_HPP
#define INTEGRATION_CONTROL_HPP

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class IntegrationControl
            {
                uint8_t action;
                int parameter;

                /**
                 * Default Constructor
                 */
                IntegrationControl() = default;

                /**
                 * Constructor that parses a message buffer
                 */
                InegrationControl(const std::string& buffer);

                /**
                 * Encodes this object into a message buffer for a Integration Control message
                 * @returns The message buffer to send
                 */
                std::string encode();

                /**
                 * Encodes this object into a message buffer for an Integration Control Response message
                 * @return [description]
                 */
                std::string encodeResponse();

            private:
                // Integration Control Command Packet
                struct IntegrationControlCommand
                {
                    static constexpr char kMsgId              = 'I';
                    static constexpr uint8_t kActionIndex     = 4;
                    static constexpr uint8_t kParameterIndex = 5;
                    static constexpr uint8_t kChecksumIndex   = 9;
                    static constexpr uint8_t kTerminatorIndex = 11;
                    static constexpr uint8_t kMessageLength   = 13;
                };

                // Integration Control Response Packet
                struct IntegrationControlResponse
                {
                    static constexpr char kMsgId              = 'j';
                    static constexpr uint8_t kActionIndex     = 4;
                    static constexpr uint8_t kParameterINdex  = 5;
                    static constexpr uint8_t kChecksumIndex   = 9;
                    static constexpr uint8_t kTerminatorIndex = 11;
                    static constexpr uint8_t kMessageLength   = 13;
                };

            };
        } // navx
    } // navigation
} // rip

#endif INTEGRATION_CONTROL_HPP