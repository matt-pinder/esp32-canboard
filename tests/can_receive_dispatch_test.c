#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "can_receive_dispatch.h"

typedef struct {
    unsigned calls;
    const void *last_frame;
} consumer_state_t;

static bool record_frame(const void *frame, void *context)
{
    consumer_state_t *state = context;
    state->calls++;
    state->last_frame = frame;
    return true;
}

int main(void)
{
    const uint32_t frame = 0x610;
    consumer_state_t relay = {0};
    consumer_state_t emulator = {0};

    can_receive_dispatch_result_t result = can_receive_dispatch_fanout(
        &frame, true, record_frame, &relay, true, record_frame, &emulator);
    assert(result.relay_called && result.relay_ok);
    assert(result.emulator_called && result.emulator_ok);
    assert(relay.calls == 1 && emulator.calls == 1);
    assert(relay.last_frame == &frame && emulator.last_frame == &frame);

    result = can_receive_dispatch_fanout(
        &frame, false, record_frame, &relay, true, record_frame, &emulator);
    assert(!result.relay_called && result.emulator_called);
    assert(relay.calls == 1 && emulator.calls == 2);

    puts("can receive dispatcher tests passed");
    return 0;
}
