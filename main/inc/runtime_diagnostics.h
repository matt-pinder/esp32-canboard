#ifndef RUNTIME_DIAGNOSTICS_H
#define RUNTIME_DIAGNOSTICS_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_system.h"

typedef struct {
    uint32_t boot_count;
    uint32_t fault_reset_count;
    esp_reset_reason_t current_reset_reason;
    const char *current_reset_reason_name;
    bool last_fault_available;
    uint32_t last_fault_boot_count;
    esp_reset_reason_t last_fault_reset_reason;
    const char *last_fault_reset_reason_name;
    bool previous_snapshot_available;
    uint32_t previous_uptime_ms;
    uint32_t previous_free_heap_bytes;
    uint32_t previous_min_free_heap_bytes;
    uint32_t previous_espnow_queue_depth;
    uint32_t previous_espnow_queue_drops;
    uint32_t previous_espnow_send_failures;
    uint32_t previous_espnow_callback_timeouts;
    bool incident_available;
    char incident[64];
    uint32_t incident_boot_count;
    uint32_t incident_uptime_ms;
    uint8_t incident_client_index;
    uint32_t incident_pending_ms;
    uint32_t incident_queue_depth;
    uint32_t incident_queue_drops;
    uint32_t incident_send_failures;
} runtime_diagnostics_status_t;

/// Initialize boot/reset persistence and the RTC health snapshot task.
esp_err_t runtime_diagnostics_init(void);

/// Return current and previous-boot diagnostic information.
void runtime_diagnostics_get_status(runtime_diagnostics_status_t *status);

/// Persist a rare ESP-NOW stall event so it survives a later power cycle.
void runtime_diagnostics_record_espnow_timeout(uint8_t client_index,
                                              uint32_t pending_ms,
                                              uint32_t queue_depth,
                                              uint32_t queue_drops,
                                              uint32_t send_failures);

#endif
