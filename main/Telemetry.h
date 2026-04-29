#pragma once

#include <esp_wifi_types_generic.h>

#define WIFI_CONNECTED_BIT BIT0
#define MQTT_STARTED_BIT   BIT1
#define MQTT_CONNECTED_BIT BIT2

const int WIFI_EVENT_USER_INIT     = WIFI_EVENT_MAX + 1;
const int WIFI_EVENT_USER_SET_SSID = WIFI_EVENT_MAX + 2;
const int WIFI_EVENT_USER_SET_PASS = WIFI_EVENT_MAX + 3;
