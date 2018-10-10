
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "nvs_flash.h"
#include "config.h"


static const char *TAG = "main";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
static uint8_t tx_buf[TX_SIZE] = { 0, };
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;

/**
 * @brief ESP Main Entry Function
 * 
 */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    //set up TCPIP and Disable DHCP
    tcpip_adapter_init();
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
    ESP_ERROR_CHECK(tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA));

    //Init WIFI with defaults
    ESP_ERROR_CHECK(esp_event_loop_init(NULL, NULL));
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    //Mesh init
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(MESH_VOTE_PERCENTAGE));
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(MESH_ASSOC_EXP));

    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    cfg.event_cb = &mesh_event_handler;
    cfg.channel = MESH_CHANNEL;
    cfg.router.ssid_len = strlen(MESH_ROUTER_SSID);
    memcpy((uint8_t *) &cfg.router.ssid, MESH_ROUTER_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *) &cfg.router.password, MESH_ROUTER_PASSWD, strlen(MESH_ROUTER_PASSWD));


    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, MESH_AP_PASSWD, strlen(MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%d, %s\n",  esp_get_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed");
}


