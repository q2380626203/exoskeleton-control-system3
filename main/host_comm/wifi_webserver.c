#include "wifi_webserver.h"
#include "webpage.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "lwip/ip4_addr.h"
#include <string.h>

static const char *TAG = "WIFI_WEBSERVER";

/* WiFi事件处理 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        (void)event_data;  // 运行时日志已禁用，避免未使用变量警告
        // wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        // ESP_LOGI(TAG, "客户端连接，MAC地址:" MACSTR ", AID=%d",
        //          MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        (void)event_data;  // 运行时日志已禁用，避免未使用变量警告
        // wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        // ESP_LOGI(TAG, "客户端断开连接，MAC地址:" MACSTR ", AID=%d",
        //          MAC2STR(event->mac), event->aid);
    }
}

esp_err_t wifi_init_softap(void)
{
    esp_err_t ret;

    /* 初始化NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 初始化TCP/IP协议栈 */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 创建默认的WiFi AP网络接口 */
    esp_netif_t *netif = esp_netif_create_default_wifi_ap();

    /* 配置静态IP */
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    esp_netif_dhcps_stop(netif);
    esp_netif_set_ip_info(netif, &ip_info);
    esp_netif_dhcps_start(netif);

    /* WiFi初始化配置 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 注册WiFi事件处理器 */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    /* WiFi AP配置 */
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASSWORD,
            .max_connection = WIFI_AP_MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    /* 如果密码为空，设置为开放模式 */
    if (strlen(WIFI_AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    /* 设置WiFi模式为AP */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 降低WiFi发射功率以减少电流消耗，防止掉电重启
    // 必须在esp_wifi_start()之后调用
    // 范围: 8 (2dBm, 最低) 到 78 (19.5dBm, 最高)
    // 默认值通常为78 (19.5dBm)，这里设置为40 (10dBm) 以降低功耗
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(40)); // 10dBm，约为默认功率的一半
    ESP_LOGI(TAG, "WiFi发射功率已设置为10dBm (降低功耗模式)");

    ESP_LOGI(TAG, "WiFi热点已启动");
    ESP_LOGI(TAG, "SSID: %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "密码: %s", WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "IP地址: 192.168.4.1");

    return ESP_OK;
}

uint32_t get_system_uptime(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

/* HTTP GET处理函数 - 主页 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, webpage_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* HTTP GET处理函数 - 命令API */
static esp_err_t command_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Command handler not implemented\"}");
    return ESP_OK;
}

/* HTTP GET处理函数 - 状态API */
static esp_err_t status_get_handler(httpd_req_t *req)
{
    char response[128];
    uint32_t uptime = get_system_uptime();

    snprintf(response, sizeof(response),
             "{\"status\":\"ok\",\"uptime\":%lu,\"ip\":\"192.168.4.1\"}",
             (unsigned long)uptime);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}

/* HTTP GET处理函数 - 参数设置API */
static esp_err_t params_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Params handler not implemented\"}");
    return ESP_OK;
}

/* HTTP GET处理函数 - 电机参数设置API */
static esp_err_t motor_params_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Motor params handler not implemented\"}");
    return ESP_OK;
}

/* HTTP GET处理函数 - 读取电机参数API */
static esp_err_t get_motor_params_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Get motor params not implemented\"}");
    return ESP_OK;
}

/* HTTP GET处理函数 - 状态机状态API */
static esp_err_t state_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"State getter not implemented\"}");
    return ESP_OK;
}

/* URI处理器配置 */
static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_command = {
    .uri       = "/api/command",
    .method    = HTTP_GET,
    .handler   = command_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_status = {
    .uri       = "/api/status",
    .method    = HTTP_GET,
    .handler   = status_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_params = {
    .uri       = "/api/params",
    .method    = HTTP_GET,
    .handler   = params_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_motor_params = {
    .uri       = "/api/motor_params",
    .method    = HTTP_GET,
    .handler   = motor_params_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_get_motor_params = {
    .uri       = "/api/get_motor_params",
    .method    = HTTP_GET,
    .handler   = get_motor_params_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_state = {
    .uri       = "/api/state",
    .method    = HTTP_GET,
    .handler   = state_get_handler,
    .user_ctx  = NULL
};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEBSERVER_PORT;
    config.lru_purge_enable = true;

    /* 启动HTTP服务器 */
    ESP_LOGI(TAG, "正在启动Web服务器，端口: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        /* 注册URI处理器 */
        ESP_LOGI(TAG, "注册URI处理器");
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &api_command);
        httpd_register_uri_handler(server, &api_params);
        httpd_register_uri_handler(server, &api_motor_params);
        httpd_register_uri_handler(server, &api_get_motor_params);
        httpd_register_uri_handler(server, &api_status);
        httpd_register_uri_handler(server, &api_state);

        ESP_LOGI(TAG, "Web服务器启动成功");
        return server;
    }

    ESP_LOGE(TAG, "Web服务器启动失败");
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    if (server) {
        httpd_stop(server);
        ESP_LOGI(TAG, "Web服务器已停止");
    }
}
