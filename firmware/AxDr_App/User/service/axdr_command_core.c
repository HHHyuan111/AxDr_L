#include "axdr_command_core.h"

#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(axdr_command_request_header_t) ==
                   AXDR_COMMAND_REQUEST_HEADER_SIZE,
               "Unexpected command request header size");
_Static_assert(sizeof(axdr_command_response_header_t) ==
                   AXDR_COMMAND_RESPONSE_HEADER_SIZE,
               "Unexpected command response header size");
_Static_assert(offsetof(axdr_command_request_header_t, sequence) == 8u,
               "Unexpected request sequence offset");
_Static_assert(offsetof(axdr_command_response_header_t, ack_state) == 14u,
               "Unexpected response ACK offset");
_Static_assert(offsetof(axdr_command_response_header_t, payload_length) == 16u,
               "Unexpected response payload length offset");

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8u);
}

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8u) |
           ((uint32_t)data[2] << 16u) |
           ((uint32_t)data[3] << 24u);
}

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8u);
}

static void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8u);
    data[2] = (uint8_t)(value >> 16u);
    data[3] = (uint8_t)(value >> 24u);
}

uint32_t axdr_command_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = AXDR_CRC32_INITIAL;

    if ((data == NULL) && (length != 0u))
    {
        return 0u;
    }

    for (uint32_t i = 0u; i < length; ++i)
    {
        crc ^= data[i];
        for (uint32_t bit = 0u; bit < 8u; ++bit)
        {
            const uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1u) ^
                  (AXDR_CRC32_REVERSED_POLYNOMIAL & mask);
        }
    }

    return crc ^ AXDR_CRC32_XOR_OUT;
}

static uint16_t build_response(uint32_t sequence,
                               uint16_t opcode,
                               uint8_t ack_state,
                               uint8_t reason,
                               const uint8_t *payload,
                               uint16_t payload_length,
                               uint8_t *response,
                               uint16_t response_capacity)
{
    const uint16_t frame_size =
        (uint16_t)(AXDR_COMMAND_RESPONSE_HEADER_SIZE +
                   payload_length + AXDR_COMMAND_CRC_SIZE);

    if ((response == NULL) ||
        (frame_size > AXDR_COMMAND_MAX_RESPONSE_SIZE) ||
        (frame_size > response_capacity))
    {
        return 0u;
    }

    memset(response, 0, frame_size);
    write_u32(&response[0], AXDR_COMMAND_RESPONSE_MAGIC);
    write_u16(&response[4], AXDR_COMMAND_VERSION);
    write_u16(&response[6], frame_size);
    write_u32(&response[8], sequence);
    write_u16(&response[12], opcode);
    response[14] = ack_state;
    response[15] = reason;
    write_u16(&response[16], payload_length);

    if ((payload_length != 0u) && (payload != NULL))
    {
        memcpy(&response[AXDR_COMMAND_RESPONSE_HEADER_SIZE],
               payload, payload_length);
    }

    write_u32(&response[frame_size - AXDR_COMMAND_CRC_SIZE],
              axdr_command_crc32(response,
                                 frame_size - AXDR_COMMAND_CRC_SIZE));
    return frame_size;
}

static uint16_t reject_request(uint32_t sequence,
                               uint16_t opcode,
                               uint8_t reason,
                               uint8_t *response,
                               uint16_t response_capacity)
{
    return build_response(sequence, opcode, AXDR_ACK_REJECTED, reason,
                          NULL, 0u, response, response_capacity);
}

void axdr_command_core_init(axdr_command_core_t *core)
{
    if (core != NULL)
    {
        memset(core, 0, sizeof(*core));
    }
}

uint16_t axdr_command_core_process(
    axdr_command_core_t *core,
    const uint8_t *request,
    uint16_t request_length,
    axdr_command_payload_handler_t handler,
    void *handler_context,
    uint8_t *response,
    uint16_t response_capacity)
{
    uint16_t version;
    uint16_t frame_size;
    uint32_t sequence;
    uint16_t opcode;
    uint16_t payload_length;
    uint16_t expected_payload_length;
    uint32_t expected_crc;
    uint32_t actual_crc;
    uint8_t response_payload[
        AXDR_COMMAND_MAX_RESPONSE_SIZE -
        AXDR_COMMAND_RESPONSE_HEADER_SIZE -
        AXDR_COMMAND_CRC_SIZE];
    uint16_t response_payload_length = 0u;
    uint8_t ack_state = AXDR_ACK_ACCEPTED;
    uint8_t reason;
    uint16_t response_length;

    if ((core == NULL) || (request == NULL) ||
        (response == NULL) ||
        (response_capacity < AXDR_COMMAND_MIN_RESPONSE_SIZE) ||
        (request_length < AXDR_COMMAND_REQUEST_HEADER_SIZE))
    {
        return 0u;
    }
    if (read_u32(&request[0]) != AXDR_COMMAND_REQUEST_MAGIC)
    {
        return 0u;
    }

    version = read_u16(&request[4]);
    frame_size = read_u16(&request[6]);
    sequence = read_u32(&request[8]);
    opcode = read_u16(&request[12]);
    payload_length = read_u16(&request[14]);

    if ((frame_size < AXDR_COMMAND_MIN_REQUEST_SIZE) ||
        (frame_size > AXDR_COMMAND_MAX_REQUEST_SIZE) ||
        (request_length != frame_size))
    {
        return reject_request(sequence, opcode, AXDR_REASON_INVALID_LENGTH,
                              response, response_capacity);
    }

    expected_payload_length =
        (uint16_t)(frame_size - AXDR_COMMAND_REQUEST_HEADER_SIZE -
                   AXDR_COMMAND_CRC_SIZE);
    if (version != AXDR_COMMAND_VERSION)
    {
        return reject_request(sequence, opcode,
                              AXDR_REASON_UNSUPPORTED_VERSION,
                              response, response_capacity);
    }
    if (payload_length != expected_payload_length)
    {
        return reject_request(sequence, opcode, AXDR_REASON_INVALID_LENGTH,
                              response, response_capacity);
    }

    expected_crc = read_u32(&request[frame_size - AXDR_COMMAND_CRC_SIZE]);
    actual_crc = axdr_command_crc32(
        request, frame_size - AXDR_COMMAND_CRC_SIZE);
    if (expected_crc != actual_crc)
    {
        return reject_request(sequence, opcode, AXDR_REASON_BAD_CRC,
                              response, response_capacity);
    }

    if ((core->valid != 0u) && (sequence == core->sequence))
    {
        if ((opcode == core->opcode) && (actual_crc == core->request_crc))
        {
            if (core->response_length > response_capacity)
            {
                return 0u;
            }
            memcpy(response, core->response, core->response_length);
            return core->response_length;
        }
        return reject_request(sequence, opcode,
                              AXDR_REASON_SEQUENCE_CONFLICT,
                              response, response_capacity);
    }

    if (handler == NULL)
    {
        reason = AXDR_REASON_UNKNOWN_OPCODE;
    }
    else
    {
        reason = handler(
            handler_context,
            opcode,
            &request[AXDR_COMMAND_REQUEST_HEADER_SIZE],
            payload_length,
            response_payload,
            sizeof(response_payload),
            &response_payload_length,
            &ack_state);
    }

    if ((reason == AXDR_REASON_NONE) &&
        (response_payload_length > sizeof(response_payload)))
    {
        reason = AXDR_REASON_INVALID_LENGTH;
        response_payload_length = 0u;
    }
    if ((reason == AXDR_REASON_NONE) &&
        (ack_state != AXDR_ACK_ACCEPTED) &&
        (ack_state != AXDR_ACK_APPLIED))
    {
        reason = AXDR_REASON_INVALID_PAYLOAD;
        response_payload_length = 0u;
    }

    if (reason == AXDR_REASON_NONE)
    {
        response_length = build_response(
            sequence, opcode, ack_state, AXDR_REASON_NONE,
            response_payload, response_payload_length,
            response, response_capacity);
    }
    else
    {
        response_length = reject_request(
            sequence, opcode, reason, response, response_capacity);
    }

    if (response_length != 0u)
    {
        core->request_crc = actual_crc;
        core->sequence = sequence;
        core->opcode = opcode;
        core->response_length = response_length;
        memcpy(core->response, response, response_length);
        core->valid = 1u;
    }
    return response_length;
}
