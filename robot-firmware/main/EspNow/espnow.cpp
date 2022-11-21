/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdlib.h>
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
#include "espnow.h"
#include "esp_private/wifi.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";
static int package_index = -1;

#if CONFIG_DEBUG_MODE
    static int last_package_index = -1;
#endif

static QueueHandle_t s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t own_mac[ESP_NOW_ETH_ALEN];

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

inline bool password_cmp(const uint8_t *data){
    for(int i = 0; i < sizeof(CONFIG_PAYLOAD_PASSWORD)-1; ++i)
        if(data[i]!=CONFIG_PAYLOAD_PASSWORD[i])
            return false;
    return true;
}

bool check_hw_addr_at_index(const uint8_t * data, int index){
    #if CONFIG_DEBUG_MODE
        ESP_LOGW(TAG, "Comparing [%d, %d] with [%d, %d]", (int) data[index*CONFIG_MIN_PKG_SIZE], (int) data[index*CONFIG_MIN_PKG_SIZE + 1], (int) own_mac[4], (int) own_mac[5]);
    #endif
    return (data[index*CONFIG_MIN_PKG_SIZE] == own_mac[4]) && (data[index*CONFIG_MIN_PKG_SIZE + 1] == own_mac[5]);
}

static int find_valid_index_in_package(const uint8_t * data){
    
    int index_found = -1;

    for(int i = 0; i < CONFIG_ROBOTS_IN_WIFI_PKG; ++i){
        if(check_hw_addr_at_index(data, i))
            index_found = i;
    }

    return index_found;

}

static void example_espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    arara_only_payload_t evt;

    #if CONFIG_DEBUG_MODE
    ESP_LOGI(TAG, "Received: %d of %d", len, sizeof(arara_payload_t));
    #endif

    if (mac_addr == NULL || data == NULL || len != sizeof(arara_payload_t)) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    // Password is valid
    if (password_cmp(data)){

        #if CONFIG_DEBUG_MODE
            ESP_LOGI(TAG, "Password is Ok!");
        #endif
        
        memcpy(&evt,
            &data[sizeof(CONFIG_PAYLOAD_PASSWORD)-1],
            sizeof(arara_only_payload_t));

        if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
            ESP_LOGW(TAG, "Send receive queue fail");
        }
    }
    
}

static void example_espnow_task(void *pvParameter)
{
    arara_only_payload_t evt;

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    #if CONFIG_DEBUG_MODE
        ESP_LOGI(TAG, "Start sending broadcast data");
    #endif

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        
        #if CONFIG_DEBUG_MODE
            ESP_LOGI(TAG, "Received:");
        #endif

        if ((package_index == -1 ) || !check_hw_addr_at_index((uint8_t*) &evt, package_index)){
            package_index = find_valid_index_in_package((uint8_t*) &evt);
            
            #if CONFIG_DEBUG_MODE
            if (package_index != last_package_index){
                if (package_index == -1 ){
                    ESP_LOGE(TAG, "Could not find index!");
                }else{
                    ESP_LOGI(TAG, "Found new valid index at %d!", package_index);
                }
                last_package_index = package_index;
            }
            #endif            
        }

        if (package_index >= 0){
            parser_params((uint8_t*) &evt[package_index * CONFIG_MIN_PKG_SIZE + 2]);
        }

        // for(int i = 0; i < sizeof(arara_only_payload_t); ++i){
        //     ESP_LOGE(TAG, "%d", (int) evt[i]);
        // }

    }
}

static esp_err_t example_espnow_init(void)
{

    s_example_espnow_queue = xQueueCreate(CONFIG_ROBOTS_IN_WIFI_PKG, sizeof(arara_only_payload_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );

    #if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
        ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
    #endif

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = (esp_now_peer_info_t*) malloc(sizeof(esp_now_peer_info_t));
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

    xTaskCreate(example_espnow_task, "example_espnow_task", 2048, NULL, 4, NULL);
    #if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
        ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
    #endif
    
    return ESP_OK;
}

void setup_esp_now(void)
{

    // Initialize NVS
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     ESP_ERROR_CHECK( nvs_flash_erase() );
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK( ret );

    example_wifi_init();
    esp_read_mac(own_mac, ESP_MAC_WIFI_STA);
    
    #if CONFIG_DEBUG_MODE
        ESP_LOGW(TAG, "My Mac is %u %u:", (int) own_mac[4], (int) own_mac[5]);
    #endif 

    example_espnow_init();
}
