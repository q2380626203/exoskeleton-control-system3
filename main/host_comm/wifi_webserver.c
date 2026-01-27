#include "wifi_webserver.h"
#include "webpage.h"
#include "speed_follow_wrapper.h"
#include "tcp_data_upload.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "lwip/ip4_addr.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

// static const char *TAG = "WIFI_WEBSERVER";  // 运行时日志已禁用，TAG未使用

/* 外部电机参数访问 */
static float* g_motor1_pos = NULL;
static float* g_motor2_pos = NULL;
static SemaphoreHandle_t g_motor_mutex = NULL;
static void* g_speed_follow = NULL;


/**
 * @brief 设置外部电机参数和互斥锁的访问接口
 */
void webserver_set_motor_access(float* motor1_pos, float* motor2_pos,
                                 SemaphoreHandle_t mutex, void* speed_follow_ptr) {
    g_motor1_pos = motor1_pos;
    g_motor2_pos = motor2_pos;
    g_motor_mutex = mutex;
    g_speed_follow = speed_follow_ptr;
}


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
    // ESP_LOGI(TAG, "WiFi发射功率已设置为10dBm (降低功耗模式)");  // 运行时日志已禁用

    // ESP_LOGI(TAG, "WiFi热点已启动");  // 运行时日志已禁用
    // ESP_LOGI(TAG, "SSID: %s", WIFI_AP_SSID);  // 运行时日志已禁用
    // ESP_LOGI(TAG, "密码: %s", WIFI_AP_PASSWORD);  // 运行时日志已禁用
    // ESP_LOGI(TAG, "IP地址: 192.168.4.1");  // 运行时日志已禁用

    return ESP_OK;
}

/* HTTP GET处理函数 - 主页 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, webpage_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}




/* HTTP GET处理函数 - 读取电机参数API */
static esp_err_t get_motor_params_handler(httpd_req_t *req)
{
    char response[300];
    float motor1_pos = 0.0f;
    float motor2_pos = 0.0f;

    if (g_motor_mutex && g_motor1_pos && g_motor2_pos) {
        if (xSemaphoreTake(g_motor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            motor1_pos = *g_motor1_pos;
            motor2_pos = *g_motor2_pos;
            xSemaphoreGive(g_motor_mutex);

            snprintf(response, sizeof(response),
                     "{\"status\":\"ok\",\"motor1_pos\":%.6f,\"motor2_pos\":%.6f}",
                     motor1_pos, motor2_pos);
        } else {
            snprintf(response, sizeof(response),
                     "{\"status\":\"error\",\"message\":\"Mutex timeout\"}");
        }
    } else {
        snprintf(response, sizeof(response),
                 "{\"status\":\"error\",\"message\":\"Motor access not initialized\"}");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}

/* HTTP GET处理函数 - 获取当前参数API */
static esp_err_t get_current_params_handler(httpd_req_t *req)
{
    char response[256];

    if (g_speed_follow) {
        float torque = speed_follow_get_current_torque(g_speed_follow);
        float phase2_torque = speed_follow_get_current_phase2_torque(g_speed_follow);

        snprintf(response, sizeof(response),
                 "{\"status\":\"ok\",\"torque\":%.2f,\"phase2_torque\":%.2f}",
                 torque, phase2_torque);
    } else {
        snprintf(response, sizeof(response),
                 "{\"status\":\"error\",\"message\":\"SpeedFollow not initialized\"}");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}

/* HTTP GET处理函数 - 调整力矩API */
static esp_err_t adjust_torque_handler(httpd_req_t *req)
{
    char response[256];
    char query[64];

    // 获取查询参数
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char action[16];
        if (httpd_query_key_value(query, "action", action, sizeof(action)) == ESP_OK) {
            bool increase = (strcmp(action, "increase") == 0);

            // 调用SpeedFollowMode的adjustTorque方法
            if (g_speed_follow) {
                float new_torque = speed_follow_adjust_torque(g_speed_follow, increase);

                snprintf(response, sizeof(response),
                         "{\"status\":\"ok\",\"torque\":%.2f}", new_torque);
            } else {
                snprintf(response, sizeof(response),
                         "{\"status\":\"error\",\"message\":\"SpeedFollow not initialized\"}");
            }
        } else {
            snprintf(response, sizeof(response),
                     "{\"status\":\"error\",\"message\":\"Missing action parameter\"}");
        }
    } else {
        snprintf(response, sizeof(response),
                 "{\"status\":\"error\",\"message\":\"No query string\"}");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}

/* HTTP GET处理函数 - 调整Kd API */
/* HTTP GET处理函数 - 调整Phase2力矩API */
static esp_err_t adjust_phase2_torque_handler(httpd_req_t *req)
{
    char response[256];
    char query[64];

    // 获取查询参数
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char action[16];
        if (httpd_query_key_value(query, "action", action, sizeof(action)) == ESP_OK) {
            bool increase = (strcmp(action, "increase") == 0);

            // 调用SpeedFollowMode的adjustPhase2Torque方法
            if (g_speed_follow) {
                float new_phase2_torque = speed_follow_adjust_phase2_torque(g_speed_follow, increase);

                snprintf(response, sizeof(response),
                         "{\"status\":\"ok\",\"phase2_torque\":%.2f}", new_phase2_torque);
            } else {
                snprintf(response, sizeof(response),
                         "{\"status\":\"error\",\"message\":\"SpeedFollow not initialized\"}");
            }
        } else {
            snprintf(response, sizeof(response),
                     "{\"status\":\"error\",\"message\":\"Missing action parameter\"}");
        }
    } else {
        snprintf(response, sizeof(response),
                 "{\"status\":\"error\",\"message\":\"No query string\"}");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}

/* HTTP GET处理函数 - 获取服务器时间API */
static esp_err_t get_server_time_handler(httpd_req_t *req)
{
    char response[256];

    // 检查时间是否已同步
    bool synced = is_time_synced();

    if (synced) {
        // 获取当前时间
        struct timeval tv;
        gettimeofday(&tv, NULL);

        // 转换为北京时间
        struct tm timeinfo;
        localtime_r(&tv.tv_sec, &timeinfo);

        // 格式化时间字符串: YYYY-MM-DD HH:MM:SS
        char time_str[32];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);

        snprintf(response, sizeof(response),
                 "{\"status\":\"ok\",\"synced\":true,\"time\":\"%s\"}", time_str);
    } else {
        snprintf(response, sizeof(response),
                 "{\"status\":\"ok\",\"synced\":false,\"time\":\"服务器未连接\"}");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}



/* URI处理器配置 */
static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_get_motor_params = {
    .uri       = "/api/get_motor_params",
    .method    = HTTP_GET,
    .handler   = get_motor_params_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_get_current_params = {
    .uri       = "/api/get_current_params",
    .method    = HTTP_GET,
    .handler   = get_current_params_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_adjust_torque = {
    .uri       = "/api/adjust_torque",
    .method    = HTTP_GET,
    .handler   = adjust_torque_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_adjust_phase2_torque = {
    .uri       = "/api/adjust_phase2_torque",
    .method    = HTTP_GET,
    .handler   = adjust_phase2_torque_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_get_server_time = {
    .uri       = "/api/get_server_time",
    .method    = HTTP_GET,
    .handler   = get_server_time_handler,
    .user_ctx  = NULL
};


httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEBSERVER_PORT;
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;

    /* 启动HTTP服务器 */
    // ESP_LOGI(TAG, "正在启动Web服务器，端口: %d", config.server_port);  // 运行时日志已禁用
    if (httpd_start(&server, &config) == ESP_OK) {
        /* 注册URI处理器 */
        // ESP_LOGI(TAG, "注册URI处理器");  // 运行时日志已禁用
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &api_get_motor_params);
        httpd_register_uri_handler(server, &api_get_current_params);
        httpd_register_uri_handler(server, &api_adjust_torque);
        httpd_register_uri_handler(server, &api_adjust_phase2_torque);
        httpd_register_uri_handler(server, &api_get_server_time);

        // ESP_LOGI(TAG, "Web服务器启动成功");  // 运行时日志已禁用
        return server;
    }

    // ESP_LOGE(TAG, "Web服务器启动失败");  // 运行时日志已禁用
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    if (server) {
        httpd_stop(server);
        // ESP_LOGI(TAG, "Web服务器已停止");  // 运行时日志已禁用
    }
}
