#include <iostream>
#include <string.h> // For strtok
#include <stdlib.h> // For atof, atoi
#include <algorithm> // For std::min, resolves 'MIN' was not declared in this scope
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // For Semaphore
#include "esp_log.h"
#include "unitree_motor.h"
#include "motor_commands.h"
#include "speed_follow_mode.h"
#include "driver/gpio.h"
#include "esp_wifi.h" // For Wi-Fi
#include "esp_event.h" // For Wi-Fi events
#include "nvs_flash.h" // For NVS (Non-Volatile Storage)
#include "esp_http_server.h" // For HTTP server
#include "esp_netif.h" // For network interface configuration
#include "lwip/ip_addr.h" // For IP4_ADDR and esp_ip4addr_ntoa

static const char *TAG = "MAIN";

// 定义电机ID和UART端口/引脚
#define MOTOR_ID        0x02
#define UART_PORT_NUM   UART_NUM_2
#define UART_TX_PIN     GPIO_NUM_13
#define UART_RX_PIN     GPIO_NUM_12
// #define MAX485_RE_DE_PIN GPIO_NUM_6  // 纯串口模式不需要DE/RE控制引脚
#define UART_BAUD_RATE  4000000

UnitreeMotorDriver motor_driver;
SpeedFollowMode speed_follow; // 速度跟随模式实例

// Motor command parameters, protected by a mutex
static uint8_t global_motor_id = MOTOR_ID;
static uint8_t global_motor_mode = 1; // 默认位置模式
static float global_motor_pos = 0.0f;
static float global_motor_vel = 0.0f;
static float global_motor_t = 0.0f;
static float global_motor_kp = 0.0f;
static float global_motor_kd = 0.0f;

SemaphoreHandle_t motor_params_mutex; // 用于保护电机参数的互斥锁

// Wi-Fi配置
#define EXAMPLE_ESP_WIFI_SSID      "MIFI-6CDF" // 更新为用户提供的SSID
#define EXAMPLE_ESP_WIFI_PASS      "1234567890" // 更新为用户提供的密码
#define EXAMPLE_ESP_MAX_RETRY      5

// 静态IP配置
#define EXAMPLE_STATIC_IP          "192.168.100.201"
#define EXAMPLE_STATIC_GW          "192.168.100.1"
#define EXAMPLE_STATIC_NETMASK     "255.255.255.0"

static int s_retry_num = 0;

// Wi-Fi事件处理器
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    // 错误修复：使用具体的事件ID，而不是WIFI_STA_EVENT
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            ESP_LOGI(TAG, "Connect to the AP failed. Check your SSID and Password.");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        // 错误修复：esp_ip4addr_ntoa需要一个缓冲区和长度参数
        char ip_str[IP4ADDR_STRLEN_MAX]; // 定义一个足够大的缓冲区来存储IP地址字符串
        ESP_LOGI(TAG, "got ip:%s", esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str)));
        s_retry_num = 0;
    }
}

// 初始化Wi-Fi并设置静态IP
void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 创建默认的Wi-Fi STA网络接口
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // 禁用DHCP客户端以设置静态IP
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta_netif));

    // 配置静态IP信息
    esp_netif_ip_info_t ip_info;
    // 错误修复：IP4_ADDR需要lwip/ip_addr.h
    IP4_ADDR(&ip_info.ip, 192, 168, 100, 201);       // 本地IP
    IP4_ADDR(&ip_info.gw, 192, 168, 100, 1);         // 网关
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);    // 子网掩码

    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));
    ESP_LOGI(TAG, "Set static IP: %s, Gateway: %s, Netmask: %s",
             EXAMPLE_STATIC_IP, EXAMPLE_STATIC_GW, EXAMPLE_STATIC_NETMASK);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, // 错误修复：使用WIFI_EVENT
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 错误修复：C++风格的结构体初始化
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold = {
                .authmode = WIFI_AUTH_WPA2_PSK,
            },
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

// 用于提供网页的HTTP GET处理器
esp_err_t get_handler(httpd_req_t *req) {
    extern const unsigned char index_html_start[] asm("_binary_index_html_start");
    extern const unsigned char index_html_end[] asm("_binary_index_html_end");
    const size_t index_html_len = index_html_end - index_html_start;

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_len);
    return ESP_OK;
}

// HTTP POST handler for updating all parameters
esp_err_t post_handler(httpd_req_t *req) {
    char content[512]; // Increased buffer size to accommodate all parameters
    int ret, remaining = req->content_len;
    int received_len = 0;

    // Read the POST data
    while (remaining > 0) {
        // 错误修复：使用std::min代替MIN
        if ((ret = httpd_req_recv(req, content + received_len, std::min(remaining, (int)(sizeof(content) - received_len -1)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Error receiving POST data: %d", ret);
            return ESP_FAIL;
        }
        received_len += ret;
        remaining -= ret;
    }
    content[received_len] = '\0'; // Null-terminate the received content

    ESP_LOGI(TAG, "Received POST data: %s", content);

    // Acquire mutex before updating global parameters
    if (xSemaphoreTake(motor_params_mutex, portMAX_DELAY) == pdTRUE) {
        char *token;
        char *rest = content;

        // Parse id
        token = strtok_r(rest, "&", &rest);
        if (token && strstr(token, "id=")) global_motor_id = (uint8_t)atoi(token + 3);

        // Parse mode
        token = strtok_r(rest, "&", &rest);
        if (token && strstr(token, "mode=")) global_motor_mode = (uint8_t)atoi(token + 5);

        // Parse pos
        token = strtok_r(rest, "&", &rest);
        if (token && strstr(token, "pos=")) global_motor_pos = atof(token + 4);

        // Parse vel
        token = strtok_r(rest, "&", &rest);
        if (token && strstr(token, "vel=")) global_motor_vel = atof(token + 4);

        // Parse t (torque)
        token = strtok_r(rest, "&", &rest);
        if (token && strstr(token, "t=")) global_motor_t = atof(token + 2);

        // Parse kp
        token = strtok_r(rest, "&", &rest);
        if (token && strstr(token, "kp=")) global_motor_kp = atof(token + 3);

        // Parse kd
        token = strtok_r(rest, "&", &rest);
        if (token && strstr(token, "kd=")) global_motor_kd = atof(token + 3);

        xSemaphoreGive(motor_params_mutex); // Release mutex
        ESP_LOGI(TAG, "Updated parameters: ID=%d, Mode=%d, Pos=%.3f, Vel=%.3f, T=%.3f, Kp=%.3f, Kd=%.3f",
                 global_motor_id, global_motor_mode, global_motor_pos, global_motor_vel,
                 global_motor_t, global_motor_kp, global_motor_kd);
    } else {
        ESP_LOGE(TAG, "Failed to acquire motor_params_mutex in post_handler!");
    }


    httpd_resp_sendstr_chunk(req, "Parameters updated successfully!");
    // httpd_resp_send_chunk_finish(req); // 此函数可能在您的版本中被弃用或需要其他上下文
    // 我们可以通过发送一个空的块来完成分块传输
    httpd_resp_send_chunk(req, NULL, 0); // 错误修复：使用 send_chunk(NULL, 0) 完成分块传输
    return ESP_OK;
}

// HTTP server configuration
httpd_uri_t uri_get = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = get_handler,
    .user_ctx = NULL
};

httpd_uri_t uri_post = {
    .uri      = "/set_params", // Changed endpoint to set_params
    .method   = HTTP_POST,
    .handler  = post_handler,
    .user_ctx = NULL
};

// Start the HTTP server
httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192; // Increase stack size if needed
    config.server_port = 80; // Explicitly set server port to 80

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_post);
        return server;
    }
    ESP_LOGE(TAG, "Failed to start HTTP server!");
    return NULL;
}

// 电机控制任务
void motor_control_task(void *pvParameters) {
    if (!motor_driver.isInitialized()) {
        ESP_LOGE(TAG, "电机驱动未初始化！");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "电机控制任务启动 - 纯串口同步模式 (500Hz)");

    // 初始化速度跟随模式
    speed_follow.init();

    // 设置速度跟随模式的全局参数访问
    speed_follow.setGlobalParams(&global_motor_id, &global_motor_mode, &global_motor_pos,
                                &global_motor_vel, &global_motor_t, &global_motor_kp,
                                &global_motor_kd, motor_params_mutex);

    // 统计计数器
    uint32_t loop_count = 0;

    MotorCmdA1 enable_cmd;
    enable_cmd.id = MOTOR_ID;
    enable_cmd.mode = 0x00;

    MotorDataA1 motor_data = {0}; // 初始化为0
    esp_err_t err;

    ESP_LOGI(TAG, "尝试使能电机 ID: 0x%02X", MOTOR_ID);
    err = motor_driver.sendRecv(enable_cmd, motor_data);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "电机 ID: 0x%02X 状态反馈：Mode=0x%02X, Pos=%.3f, Vel=%.3f, T=%.3f, Temp=%d, Error=0x%02X",
                 motor_data.id, motor_data.mode, motor_data.pos, motor_data.vel, motor_data.t, motor_data.temp, motor_data.MError);
    } else {
        ESP_LOGE(TAG, "使能电机 ID: 0x%02X 失败，错误码: %d", MOTOR_ID, err);
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Local variables to hold the current parameters
    uint8_t current_motor_id;
    uint8_t current_motor_mode;
    float current_motor_pos;
    float current_motor_vel;
    float current_motor_t;
    float current_motor_kp;
    float current_motor_kd;

    while (1) {
        // Safely read the global parameter values
        if (xSemaphoreTake(motor_params_mutex, portMAX_DELAY) == pdTRUE) {
            current_motor_id = global_motor_id;
            current_motor_mode = global_motor_mode;
            current_motor_pos = global_motor_pos;
            current_motor_vel = global_motor_vel;
            current_motor_t = global_motor_t;
            current_motor_kp = global_motor_kp;
            current_motor_kd = global_motor_kd;
            xSemaphoreGive(motor_params_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to take motor_params_mutex, using default parameters.");
            // Fallback to default values if mutex acquisition fails
            current_motor_id = MOTOR_ID;
            current_motor_mode = 0;
            current_motor_pos = 0.0f;
            current_motor_vel = 0.0f;
            current_motor_t = 0.0f;
            current_motor_kp = 0.0f;
            current_motor_kd = 0.0f;
        }

        MotorCmdA1 control_cmd;
        control_cmd.id = current_motor_id;
        control_cmd.mode = current_motor_mode;
        control_cmd.pos = current_motor_pos;
        control_cmd.vel = current_motor_vel;
        control_cmd.t = current_motor_t;
        control_cmd.kp = current_motor_kp;
        control_cmd.kd = current_motor_kd;

        // 先发送一次获取电机状态
        err = motor_driver.sendRecv(control_cmd, motor_data);

        // 速度跟随模式处理 - 如果触发，会修改全局参数
        if (err == ESP_OK) {
            speed_follow.update(motor_data);
        }

        loop_count++;

        // 关闭详细状态打印，只保留格式化的电机数据输出
        // 数据已经在 sendRecv 中按 "motor2:pos,vel,torque" 格式输出

        // 高频率控制: 2ms延时 = 500Hz控制频率
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "ESP32 Unitree 电机驱动示例启动");

    // Initialize NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 为电机参数创建互斥锁
    motor_params_mutex = xSemaphoreCreateMutex();
    if (motor_params_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create motor_params_mutex!");
        return;
    }

    // WiFi和HTTP服务器功能已禁用

    if (motor_driver.init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, GPIO_NUM_NC, UART_BAUD_RATE)) {
        ESP_LOGI(TAG, "电机驱动初始化成功 - 纯串口同步模式");
    } else {
        ESP_LOGE(TAG, "电机驱动初始化失败！");
        return;
    }

    xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);
}
