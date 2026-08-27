#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

/// @brief Start HTTP server with REST API endpoints and SPIFFS mount
/// Serves index.min.html.gz from SPIFFS and provides REST API for:
/// - GET /api/config: Retrieve aggregate board and output-rule configuration
/// - POST /api/config: Validate, persist, and apply the whole configuration
/// - POST /api/reboot: Reboot the device
/// - GET /api/ntc_tables: List available NTC lookup tables
void start_http_server(void);

/// @brief Stop HTTP server and unmount SPIFFS
/// Called after the last AP client disconnects or during shutdown.
void stop_http_server(void);

#endif // HTTP_SERVER_H
