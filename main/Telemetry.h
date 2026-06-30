#pragma once

#include <esp_wifi_types_generic.h>

#define WIFI_CONNECTED_BIT BIT0
#define MQTT_STARTED_BIT   BIT1

const int WIFI_EVENT_USER_INIT         = WIFI_EVENT_MAX + 1;
const int WIFI_EVENT_USER_UPDATE_CREDS = WIFI_EVENT_MAX + 2;
