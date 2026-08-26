#include "inc/can_receive_dispatch.h"

#include <stddef.h>

can_receive_dispatch_result_t can_receive_dispatch_fanout(
    const void *frame,
    bool relay_enabled,
    can_receive_consumer_t relay_consumer,
    void *relay_context,
    bool emulator_enabled,
    can_receive_consumer_t emulator_consumer,
    void *emulator_context)
{
    can_receive_dispatch_result_t result = {0};
    if (frame == NULL) return result;

    if (relay_enabled && relay_consumer != NULL) {
        result.relay_called = true;
        result.relay_ok = relay_consumer(frame, relay_context);
    }
    if (emulator_enabled && emulator_consumer != NULL) {
        result.emulator_called = true;
        result.emulator_ok = emulator_consumer(frame, emulator_context);
    }
    return result;
}
