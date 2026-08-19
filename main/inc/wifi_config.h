#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

/// @brief Connect to Broomhall IoT, falling back to the local configuration AP.
/// Searches for the preferred network for 10 seconds. If unavailable, starts
/// ESP32-CanBoard and its HTTP server with the normal 120-second timeout.
void wifi_config_mode_start(void);

/// @brief Notify that a client has connected and reset inactivity timer
/// Call this from HTTP server connection handler to prevent timeout during active configuration.
void notify_client_connected(void);

#endif // WIFI_CONFIG_H
