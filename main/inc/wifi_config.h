#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

/// @brief Initialize WiFi AP mode with configuration timeout
/// Starts WiFi access point (SSID: ESP32-CanBoard) and HTTP configuration server.
/// Automatically shuts down after 60 seconds of inactivity.
void wifi_config_mode_start(void);

/// @brief Notify that a client has connected and reset inactivity timer
/// Call this from HTTP server connection handler to prevent timeout during active configuration.
void notify_client_connected(void);

#endif // WIFI_CONFIG_H
