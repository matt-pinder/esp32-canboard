#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

/// @brief Keep the local configuration AP active and optionally connect to preferred WiFi.
/// The HTTP server runs only while a station is associated with the AP.
void wifi_config_mode_start(void);

/// @brief Compatibility hook retained for existing HTTP request handlers.
void notify_client_connected(void);

#endif // WIFI_CONFIG_H
