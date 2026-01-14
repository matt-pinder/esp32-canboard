#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

/// @brief Start HTTP server with REST API endpoints and SPIFFS mount
/// Serves index.html from SPIFFS and provides REST API for:
/// - GET /api/config: Retrieve current board configuration
/// - POST /api/config: Update board configuration
/// - POST /api/reboot: Reboot the device
/// - GET /api/ntc_tables: List available NTC lookup tables
void start_http_server(void);

/// @brief Stop HTTP server and unmount SPIFFS
/// Called on WiFi timeout or shutdown to cleanly release resources.
void stop_http_server(void);

#endif // HTTP_SERVER_H
