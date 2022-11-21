#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "parser.h"
#include "freertos/FreeRTOS.h"

#include <definitions.h>

void esp_spp_cb(esp_spp_cb_event_t , esp_spp_cb_param_t *);
void esp_bt_gap_cb(esp_bt_gap_cb_event_t , esp_bt_gap_cb_param_t *);
void setup_bluetooth();
