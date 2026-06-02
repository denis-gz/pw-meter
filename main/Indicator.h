#pragma once

enum led_mode_t {
    LED_STATE_OFF,
    LED_STATE_WIFI_SEARCHING, // Persistent slow blink
    LED_STATE_CONNECTED,      // Persistent steady ON
    LED_STATE_MQTT_TX,        // Transient: 1 quick flash, returns to background state
    LED_STATE_MQTT_ERROR      // Transient: 3 fast flashes, returns to background state
};

