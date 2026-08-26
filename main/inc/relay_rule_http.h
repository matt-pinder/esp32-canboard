#pragma once

#include "esp_http_server.h"

/* Registers GET/POST /api/rules and GET /api/rules/status. */
void relay_rule_http_register(httpd_handle_t server);
