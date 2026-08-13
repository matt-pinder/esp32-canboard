#ifndef DRAGY_GPS_H
#define DRAGY_GPS_H

#include "esp_err.h"

/// Start the independent Dragy BLE connection and GPS frame publisher task.
esp_err_t dragy_gps_start(void);

/// Apply the current board_cfg GPS settings without restarting the ADC task.
void dragy_gps_apply_config(void);

#endif // DRAGY_GPS_H
