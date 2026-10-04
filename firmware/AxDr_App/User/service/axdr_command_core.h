#ifndef AXDR_COMMAND_CORE_H
#define AXDR_COMMAND_CORE_H

#include <stdint.h>

#include "axdr_command_contract.h"

typedef uint8_t (*axdr_command_payload_handler_t)(
    void *context,
    uint16_t opcode,
    const uint8_t *request_payload,
    uint16_t request_payload_length,
    uint8_t *response_payload,
    uint16_t response_payload_capacity,
    uint16_t *response_payload_length,
    uint8_t *ack_state);

typedef struct
{
    uint32_t request_crc;
    uint32_t sequence;
    uint16_t opcode;
    uint16_t response_length;
    uint8_t response[AXDR_COMMAND_MAX_RESPONSE_SIZE];
    uint8_t valid;
} axdr_command_core_t;

void axdr_command_core_init(axdr_command_core_t *core);

uint32_t axdr_command_crc32(const uint8_t *data, uint32_t length);

uint16_t axdr_command_core_process(
    axdr_command_core_t *core,
    const uint8_t *request,
    uint16_t request_length,
    axdr_command_payload_handler_t handler,
    void *handler_context,
    uint8_t *response,
    uint16_t response_capacity);

#endif /* AXDR_COMMAND_CORE_H */
