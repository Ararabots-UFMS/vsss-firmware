#if CONFIG_BT_ENABLED
/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
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

/*
 * DEFINES
 ****************************************************************************************
 */

#define HRPS_HT_MEAS_MAX_LEN            (13)

#define HRPS_MANDATORY_MASK             (0x0F)
#define HRPS_BODY_SENSOR_LOC_MASK       (0x30)
#define HRPS_HR_CTNL_PT_MASK            (0xC0)

#define GATTS_TABLE_TAG "SEC_GATTS_DEMO"

#define ROBOT_PROFILE_NUM                         1
#define ROBOT_PROFILE_APP_IDX                     0

#define ESP_ROBOT_APP_ID                     0x55
#define ROBOT_SVC_INST_ID                    0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX               0x40

#define ADV_CONFIG_FLAG                           (1 << 0)
#define SCAN_RSP_CONFIG_FLAG                      (1 << 1)

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

///Attributes State Machine
enum
{
    ROBOT_IDX_SVC,

    MOTOR_POWER_LEVEL_CHAR,
    MOTOR_POWER_LEVEL_VAL,

    BATTERY_LEVEL_CHAR,
    BATTERY_LEVEL_VAL,

    ROBOT_IDX_NB,
};

void setup_bluetooth();

#endif