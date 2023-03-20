/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

#include "parser.h"

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   WIFI_IF_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   WIFI_IF_AP
#endif

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

enum {
    EXAMPLE_ESPNOW_DATA_BROADCAST,
    EXAMPLE_ESPNOW_DATA_UNICAST,
    EXAMPLE_ESPNOW_DATA_MAX,
};

// ROBOTS_IN_WIFI_PKG * MIN_PKG_SIZE + PASSWORD_SIZE
typedef uint8_t arara_payload_t[CONFIG_ROBOTS_IN_WIFI_PKG * CONFIG_MIN_PKG_SIZE + sizeof(CONFIG_PAYLOAD_PASSWORD) - 1];

typedef uint8_t arara_only_payload_t[CONFIG_ROBOTS_IN_WIFI_PKG * CONFIG_MIN_PKG_SIZE];

void setup_esp_now();

#endif
