#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "nvs_flash.h"
#include "config.h"



#define RX_SIZE          (1500)
#define TX_SIZE          (1460)


static const char *TAG = "main";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
static uint8_t tx_buf[TX_SIZE] = { 0, };
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;

void esp_mesh_p2p_tx_main(void *arg)
{
    int i;
    esp_err_t err;
    int send_count = 0;
    mesh_addr_t route_table[MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    mesh_data_t data;
    data.data = tx_buf;
    data.size = sizeof(tx_buf);
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_DEF;

    is_running = true;
    while (is_running) {
        /* non-root do nothing but print */
        if (!esp_mesh_is_root()) {
            ESP_LOGI(TAG, "layer:%d, rtableSize:%d, %s", mesh_layer,
                     esp_mesh_get_routing_table_size(),
                     (is_mesh_connected && esp_mesh_is_root()) ? "ROOT" : is_mesh_connected ? "NODE" : "DISCONNECT");
            vTaskDelay(10 * 1000 / portTICK_RATE_MS);
            continue;
        }

        esp_mesh_get_routing_table((mesh_addr_t *) &route_table,
                                   MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);
        if (send_count && !(send_count % 100)) {
            ESP_LOGI(TAG, "size:%d/%d,send_count:%d", route_table_size,
                     esp_mesh_get_routing_table_size(), send_count);
        }
        send_count++;
        tx_buf[25] = (send_count >> 24) & 0xff;
        tx_buf[24] = (send_count >> 16) & 0xff;
        tx_buf[23] = (send_count >> 8) & 0xff;
        tx_buf[22] = (send_count >> 0) & 0xff;
        /*
        if (send_count % 2) {
            memcpy(tx_buf, (uint8_t *)&light_on, sizeof(light_on));
        } else {
            memcpy(tx_buf, (uint8_t *)&light_off, sizeof(light_off));
        }*/

        for (i = 0; i < route_table_size; i++) {
            err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
            if (err) {
                ESP_LOGE(TAG,
                         "[ROOT-2-UNICAST:%d][L:%d]parent:"MACSTR" to "MACSTR", heap:%d[err:0x%x, proto:%d, tos:%d]",
                         send_count, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                         MAC2STR(route_table[i].addr), esp_get_free_heap_size(),
                         err, data.proto, data.tos);
            } else if (!(send_count % 100)) {
                ESP_LOGW(TAG,
                         "[ROOT-2-UNICAST:%d][L:%d][rtableSize:%d]parent:"MACSTR" to "MACSTR", heap:%d[err:0x%x, proto:%d, tos:%d]",
                         send_count, mesh_layer,
                         esp_mesh_get_routing_table_size(),
                         MAC2STR(mesh_parent_addr.addr),
                         MAC2STR(route_table[i].addr), esp_get_free_heap_size(),
                         err, data.proto, data.tos);
            }
        }
        /* if route_table_size is less than 10, add delay to avoid watchdog in this task. */
        if (route_table_size < 10) {
            vTaskDelay(1 * 1000 / portTICK_RATE_MS);
        }
    }
    vTaskDelete(NULL);
}

void esp_mesh_p2p_rx_main(void *arg)
{
    int recv_count = 0;
    esp_err_t err;
    mesh_addr_t from;
    int send_count = 0;
    mesh_data_t data;
    int flag = 0;
    data.data = rx_buf;
    data.size = RX_SIZE;

    is_running = true;
    while (is_running) {
        data.size = RX_SIZE;
        err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (err != ESP_OK || !data.size) {
            ESP_LOGE(TAG, "err:0x%x, size:%d", err, data.size);
            continue;
        }
        /* extract send count */
        if (data.size >= sizeof(send_count)) {
            send_count = (data.data[25] << 24) | (data.data[24] << 16)
                         | (data.data[23] << 8) | data.data[22];
        }
        recv_count++;
        if (!(recv_count % 1)) {
            ESP_LOGW(TAG,
                     "[#RX:%d/%d][L:%d] parent:"MACSTR", receive from "MACSTR", size:%d, heap:%d, flag:%d[err:0x%x, proto:%d, tos:%d]",
                     recv_count, send_count, mesh_layer,
                     MAC2STR(mesh_parent_addr.addr), MAC2STR(from.addr),
                     data.size, esp_get_free_heap_size(), flag, err, data.proto,
                     data.tos);
        }
    }
    vTaskDelete(NULL);
}

esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
    }
    return ESP_OK;
}

void mesh_event_handler(mesh_event_t event)
{
    mesh_addr_t id = {0,};
    static uint8_t last_layer = 0;
    ESP_LOGD(TAG, "esp_event_handler:%d", event.id);

    switch (event.id) {
    case MESH_EVENT_STARTED:
        esp_mesh_get_id(&id);
        ESP_LOGI(TAG, "<MESH_EVENT_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
        break;
    case MESH_EVENT_STOPPED:
        ESP_LOGI(TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
        break;
    case MESH_EVENT_CHILD_CONNECTED:
        ESP_LOGI(TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 event.info.child_connected.aid,
                 MAC2STR(event.info.child_connected.mac));
        break;
    case MESH_EVENT_CHILD_DISCONNECTED:
        ESP_LOGI(TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 event.info.child_disconnected.aid,
                 MAC2STR(event.info.child_disconnected.mac));
        break;
    case MESH_EVENT_ROUTING_TABLE_ADD:
        ESP_LOGW(TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d",
                 event.info.routing_table.rt_size_change,
                 event.info.routing_table.rt_size_new);
        break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE:
        ESP_LOGW(TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d",
                 event.info.routing_table.rt_size_change,
                 event.info.routing_table.rt_size_new);
        break;
    case MESH_EVENT_NO_PARENT_FOUND:
        ESP_LOGI(TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 event.info.no_parent.scan_times);
        /* TODO handler for the failure */
        break;
    case MESH_EVENT_PARENT_CONNECTED:
        esp_mesh_get_id(&id);
        mesh_layer = event.info.connected.self_layer;
        memcpy(&mesh_parent_addr.addr, event.info.connected.connected.bssid, 6);
        ESP_LOGI(TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR"",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr));
        last_layer = mesh_layer;
        //mesh_connected_indicator(mesh_layer);
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA);
        }
        esp_mesh_comm_p2p_start();
        break;
    case MESH_EVENT_PARENT_DISCONNECTED:
        ESP_LOGI(TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 event.info.disconnected.reason);
        is_mesh_connected = false;
        //mesh_disconnected_indicator();
        mesh_layer = esp_mesh_get_layer();
        break;
    case MESH_EVENT_LAYER_CHANGE:
        mesh_layer = event.info.layer_change.new_layer;
        ESP_LOGI(TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
        //mesh_connected_indicator(mesh_layer);
        break;
    case MESH_EVENT_ROOT_ADDRESS:
        ESP_LOGI(TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(event.info.root_addr.addr));
        break;
    case MESH_EVENT_ROOT_GOT_IP:
        /* root starts to connect to server */
        ESP_LOGI(TAG,
                 "<MESH_EVENT_ROOT_GOT_IP>sta ip: " IPSTR ", mask: " IPSTR ", gw: " IPSTR,
                 IP2STR(&event.info.got_ip.ip_info.ip),
                 IP2STR(&event.info.got_ip.ip_info.netmask),
                 IP2STR(&event.info.got_ip.ip_info.gw));
        break;
    case MESH_EVENT_ROOT_LOST_IP:
        ESP_LOGI(TAG, "<MESH_EVENT_ROOT_LOST_IP>");
        break;
    case MESH_EVENT_VOTE_STARTED:
        ESP_LOGI(TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 event.info.vote_started.attempts,
                 event.info.vote_started.reason,
                 MAC2STR(event.info.vote_started.rc_addr.addr));
        break;
    case MESH_EVENT_VOTE_STOPPED:
        ESP_LOGI(TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    case MESH_EVENT_ROOT_SWITCH_REQ:
        ESP_LOGI(TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 event.info.switch_req.reason,
                 MAC2STR( event.info.switch_req.rc_addr.addr));
        break;
    case MESH_EVENT_ROOT_SWITCH_ACK:
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
        break;
    case MESH_EVENT_TODS_STATE:
        ESP_LOGI(TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d",
                 event.info.toDS_state);
        break;
    case MESH_EVENT_ROOT_FIXED:
        ESP_LOGI(TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 event.info.root_fixed.is_fixed ? "fixed" : "not fixed");
        break;
    case MESH_EVENT_ROOT_ASKED_YIELD:
        ESP_LOGI(TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(event.info.root_conflict.addr),
                 event.info.root_conflict.rssi,
                 event.info.root_conflict.capacity);
        break;
    case MESH_EVENT_CHANNEL_SWITCH:
        ESP_LOGI(TAG, "<MESH_EVENT_CHANNEL_SWITCH>");
        break;
    case MESH_EVENT_SCAN_DONE:
        ESP_LOGI(TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 event.info.scan_done.number);
        break;
    default:
        ESP_LOGI(TAG, "unknown id:%d", event.id);
        break;
    }
}


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


    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(WIFI_AUTH_WPA_WPA2_PSK));
    cfg.mesh_ap.max_connection = MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, MESH_AP_PASSWD, strlen(MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI(TAG, "mesh starts successfully");
}


