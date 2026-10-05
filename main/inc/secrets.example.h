#pragma once

/* Copy this file to secrets.h. Known infrastructure networks are tried in the
 * order listed when ESP-NOW is disabled.
 *
 * Password may be an empty string for an intentionally open known network.
 */
#define KNOWN_WIFI_NETWORKS(X) \
    X("REPLACE_WITH_WIFI_SSID", "REPLACE_WITH_WIFI_PASSWORD")
