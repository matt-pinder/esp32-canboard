#ifndef CAN_RECEIVE_DISPATCH_H
#define CAN_RECEIVE_DISPATCH_H

#include <stdbool.h>

typedef bool (*can_receive_consumer_t)(const void *frame, void *context);

typedef struct {
    bool relay_called;
    bool relay_ok;
    bool emulator_called;
    bool emulator_ok;
} can_receive_dispatch_result_t;

can_receive_dispatch_result_t can_receive_dispatch_fanout(
    const void *frame,
    bool relay_enabled,
    can_receive_consumer_t relay_consumer,
    void *relay_context,
    bool emulator_enabled,
    can_receive_consumer_t emulator_consumer,
    void *emulator_context);

#endif
