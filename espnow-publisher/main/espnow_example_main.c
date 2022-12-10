/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"
#include "esp_private/wifi.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";

#define ECHO_TEST_TXD 4
#define ECHO_TEST_RXD 3
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define BUF_SIZE (1024)

static QueueHandle_t s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void example_espnow_deinit();

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_internal_set_fix_rate(ESPNOW_WIFI_IF, 1, WIFI_PHY_RATE_MCS6_LGI));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    unsigned char data_buffer_counter = 0;
    uint8_t *data[16];
    for(int i = 0; i < 16; ++i)
        data[i] = (uint8_t *) malloc(BUF_SIZE);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1) {
        // Read data from the UART, wait 16 ms for busy loop
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data[data_buffer_counter], (BUF_SIZE - 1), 16 / portTICK_PERIOD_MS);

        if (len==sizeof(arara_only_payload_t)) {
            
            // ESP_LOGI(TAG, "Recv str: %s", (char *) data[data_buffer_counter]);
            
            if (xQueueSend(s_example_espnow_queue, data[data_buffer_counter], ESPNOW_MAXDELAY) != pdTRUE) {
                ESP_LOGW(TAG, "Send receive queue fail");
                // free(recv_cb->data);
            }

            data_buffer_counter = (data_buffer_counter+1) & 15; // Keeps counter between 0-15   
        }
    }
}

static void example_espnow_task(void *arg)
{

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    arara_payload_t send_param ;
    arara_only_payload_t payload;

    memset(send_param, 0x00, sizeof(send_param));

    #if CONFIG_DEBUG_MODE    
        ESP_LOGE(TAG, "Im sending: %d", sizeof(arara_payload_t));
        for(int i = 0; i < sizeof(arara_payload_t); ++i){
            ESP_LOGE(TAG, "%c", (int) send_param[i]);
        }
    #endif

    memcpy(send_param, CONFIG_PAYLOAD_PASSWORD, sizeof(CONFIG_PAYLOAD_PASSWORD) - 1);

    while (xQueueReceive(s_example_espnow_queue, &payload, portMAX_DELAY) == pdTRUE) {

        memcpy(&send_param[sizeof(CONFIG_PAYLOAD_PASSWORD)-1],
            &payload,
            sizeof(arara_only_payload_t));
        
        #if CONFIG_DEBUG_MODE
            ESP_LOGE(TAG, "Im sending:");

            for(int i = 0; i < sizeof(arara_payload_t); ++i){
                ESP_LOGE(TAG, "%d", (int) send_param[i] );
            }
        #endif

        /* Send the next data after the previous data is sent. */
        if (esp_now_send(s_example_broadcast_mac, send_param, sizeof(arara_payload_t)) != ESP_OK) {
            ESP_LOGE(TAG, "Send error");
            example_espnow_deinit();
            vTaskDelete(NULL);
        }            
    }
}

static esp_err_t example_espnow_init(void)
{

    s_example_espnow_queue = xQueueCreate(CONFIG_ROBOTS_IN_WIFI_PKG*2, sizeof(arara_only_payload_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );

#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);    

    xTaskCreate(example_espnow_task, "example_espnow_task", 4096, NULL, 4, NULL);
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    
    return ESP_OK;
}

static void example_espnow_deinit()
{
    vSemaphoreDelete(s_example_espnow_queue);
    esp_now_deinit();
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    example_wifi_init();
    example_espnow_init();
}
