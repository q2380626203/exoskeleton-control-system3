#include "tcp_data_upload.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "TCP_UPLOAD";

/* WiFi事件标志 */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;

/* TCP连接状态 */
static int s_tcp_socket = -1;
static bool s_tcp_connected = false;
static bool s_upload_running = false;
static TaskHandle_t s_upload_task_handle = NULL;

/* 数据缓冲区 - 多缓冲（每个缓冲区存200ms数据，共5个=1秒缓存） */
#define BUFFER_SIZE TCP_SAMPLES_PER_PACKET
#define BUFFER_COUNT 5  /* 5个缓冲区：可缓存1秒数据 */
static motor_data_record_t s_buffers[BUFFER_COUNT][BUFFER_SIZE];
static volatile int s_write_buf_idx = 0;   /* 当前写入的缓冲区索引 */
static volatile int s_write_index = 0;     /* 当前缓冲区内的写入位置 */
static volatile int s_send_buf_idx = 0;    /* 当前发送的缓冲区索引 */
static volatile int s_pending_count = 0;   /* 待发送的缓冲区数量 */
static SemaphoreHandle_t s_buffer_mutex = NULL;

/* 统计信息 */
static uint32_t s_packets_sent = 0;
static uint32_t s_packets_failed = 0;
static uint32_t s_bytes_sent = 0;
static uint16_t s_seq_number = 0;

/* NTP时间同步状态 */
static volatile bool s_time_synced = false;

/* WiFi事件处理 - 仅设置标志，不阻塞 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_tcp_connected = false;
        s_retry_num++;
        ESP_LOGI(TAG, "WiFi断线，尝试重连... (第%d次)", s_retry_num);
        esp_wifi_connect();  // 自动重连
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "获取到IP地址: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_init_sta(void)
{
    esp_err_t ret;

    /* 初始化NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    /* 初始化TCP/IP协议栈 */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* WiFi初始化配置 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 注册事件处理器 */
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    /* WiFi STA配置 */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_STA_SSID,
            .password = WIFI_STA_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA初始化完成，正在连接到 %s ...", WIFI_STA_SSID);

    /* 等待连接成功或失败 - 使用超时而非无限等待 */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(10000));  // 10秒超时

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi连接成功: SSID=%s", WIFI_STA_SSID);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "WiFi连接失败: SSID=%s", WIFI_STA_SSID);
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "WiFi连接超时，将在后台继续尝试");
    return ESP_ERR_TIMEOUT;
}

/* NTP时间同步回调 */
static void time_sync_notification_cb(struct timeval *tv)
{
    s_time_synced = true;

    time_t now = tv->tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    ESP_LOGI(TAG, "NTP时间同步成功: %04d-%02d-%02d %02d:%02d:%02d (北京时间)",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

void ntp_time_sync_init(void)
{
    ESP_LOGI(TAG, "初始化SNTP时间同步...");

    /* 设置时区为北京时间 (UTC+8) */
    setenv("TZ", BEIJING_TIMEZONE, 1);
    tzset();

    /* 配置SNTP */
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, NTP_SERVER_1);
    esp_sntp_setservername(1, NTP_SERVER_2);
    esp_sntp_setservername(2, NTP_SERVER_3);
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();

    ESP_LOGI(TAG, "SNTP已启动，等待时间同步...");
}

uint64_t get_beijing_timestamp_ms(void)
{
    if (!s_time_synced) {
        return 0;  /* 时间未同步，返回0 */
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);

    /* 返回毫秒时间戳 */
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)tv.tv_usec / 1000ULL;
}

double get_timestamp_double(void)
{
    if (!s_time_synced) {
        return 0.0;  /* 时间未同步，返回0让服务端过滤 */
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);

    /* 返回 秒.毫秒 格式的double */
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

bool is_time_synced(void)
{
    return s_time_synced;
}

/* WiFi后台初始化任务 - 不阻塞主程序 */
static void wifi_background_init_task(void *pvParameters)
{
    ESP_LOGI(TAG, "WiFi后台初始化任务启动");

    /* 初始化WiFi */
    esp_err_t wifi_ret = wifi_init_sta();

    /* WiFi连接成功后启动NTP时间同步 */
    if (wifi_ret == ESP_OK) {
        ntp_time_sync_init();
    }

    /* 初始化TCP上传模块 */
    if (tcp_data_upload_init() == ESP_OK) {
        /* 启动TCP上传任务 */
        tcp_data_upload_start();

        if (wifi_ret == ESP_OK) {
            ESP_LOGI(TAG, "WiFi和TCP数据上传初始化完成");
        } else {
            ESP_LOGW(TAG, "WiFi连接未成功，TCP上传任务已启动，将自动重连");
        }
    } else {
        ESP_LOGE(TAG, "TCP上传模块初始化失败");
    }

    vTaskDelete(NULL);
}

void wifi_tcp_init_background(void)
{
    /* 创建低优先级的后台初始化任务 */
    xTaskCreate(wifi_background_init_task, "wifi_bg_init", 4096, NULL, 2, NULL);
    ESP_LOGI(TAG, "WiFi/TCP后台初始化任务已创建");
}

/* TCP连接到服务器 */
static esp_err_t tcp_connect_to_server(void)
{
    struct hostent *hp;
    struct sockaddr_in server_addr;

    /* 关闭旧连接 */
    if (s_tcp_socket >= 0) {
        close(s_tcp_socket);
        s_tcp_socket = -1;
    }

    /* DNS解析 */
    ESP_LOGI(TAG, "解析服务器地址: %s", TCP_SERVER_HOST);
    hp = gethostbyname(TCP_SERVER_HOST);
    if (hp == NULL) {
        ESP_LOGE(TAG, "DNS解析失败: %s", TCP_SERVER_HOST);
        return ESP_FAIL;
    }

    /* 创建Socket */
    s_tcp_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s_tcp_socket < 0) {
        ESP_LOGE(TAG, "创建Socket失败: %d", errno);
        return ESP_FAIL;
    }

    /* 设置服务器地址 */
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_SERVER_PORT);
    memcpy(&server_addr.sin_addr, hp->h_addr, hp->h_length);

    /* 连接服务器 */
    ESP_LOGI(TAG, "连接服务器: %s:%d", TCP_SERVER_HOST, TCP_SERVER_PORT);
    if (connect(s_tcp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "连接服务器失败: %d", errno);
        close(s_tcp_socket);
        s_tcp_socket = -1;
        return ESP_FAIL;
    }

    /* 设置发送超时 */
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(s_tcp_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    /* 禁用Nagle算法，减少延迟 */
    int flag = 1;
    setsockopt(s_tcp_socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    s_tcp_connected = true;
    ESP_LOGI(TAG, "TCP服务器连接成功");
    return ESP_OK;
}

/* 发送数据包 - 使用静态缓冲区避免栈溢出 */
static tcp_data_packet_t s_send_packet;  /* 静态发送缓冲区 */

static esp_err_t tcp_send_packet(void)
{
    if (!s_tcp_connected || s_tcp_socket < 0) {
        return ESP_FAIL;
    }

    s_send_packet.magic = 0xAA55;
    s_send_packet.device_id = DEVICE_ID;
    s_send_packet.version = 5;  /* 协议版本5: int64时间戳 + int16电机数据(×100) */
    s_send_packet.seq = s_seq_number++;
    s_send_packet.count = TCP_SAMPLES_PER_PACKET;

    /* 从发送缓冲区复制数据到包中，float转int16 (×100) */
    motor_data_record_t *send_buf = s_buffers[s_send_buf_idx];
    for (int i = 0; i < TCP_SAMPLES_PER_PACKET; i++) {
        s_send_packet.samples[i].timestamp_ms = send_buf[i].timestamp_ms;
        /* 电机数据 ×100 转int16，精度0.01 */
        s_send_packet.samples[i].channels[0] = (int16_t)(send_buf[i].motor1_pos * 100.0f);
        s_send_packet.samples[i].channels[1] = (int16_t)(send_buf[i].motor1_vel * 100.0f);
        s_send_packet.samples[i].channels[2] = (int16_t)(send_buf[i].motor1_torque * 100.0f);
        s_send_packet.samples[i].channels[3] = (int16_t)(send_buf[i].motor2_pos * 100.0f);
        s_send_packet.samples[i].channels[4] = (int16_t)(send_buf[i].motor2_vel * 100.0f);
        s_send_packet.samples[i].channels[5] = (int16_t)(send_buf[i].motor2_torque * 100.0f);
        s_send_packet.samples[i].m1_state = send_buf[i].m1_state_label;
        s_send_packet.samples[i].m2_state = send_buf[i].m2_state_label;
    }

    /* 发送数据 */
    int sent = send(s_tcp_socket, &s_send_packet, sizeof(s_send_packet), 0);
    if (sent < 0) {
        ESP_LOGE(TAG, "发送失败: %d", errno);
        s_tcp_connected = false;
        s_packets_failed++;
        return ESP_FAIL;
    }

    s_packets_sent++;
    s_bytes_sent += sent;
    return ESP_OK;
}

/* 上传任务 */
static void tcp_upload_task(void *pvParameters)
{
    ESP_LOGI(TAG, "TCP上传任务启动");

    while (s_upload_running) {
        /* 检查TCP连接 */
        if (!s_tcp_connected) {
            ESP_LOGI(TAG, "尝试重新连接...");
            if (tcp_connect_to_server() != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(3000));  // 重连间隔3秒
                continue;
            }
        }

        /* 检查是否有数据待发送 */
        if (s_pending_count > 0) {
            if (tcp_send_packet() == ESP_OK) {
                /* 发送成功，移动到下一个缓冲区 */
                if (xSemaphoreTake(s_buffer_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    s_send_buf_idx = (s_send_buf_idx + 1) % BUFFER_COUNT;
                    s_pending_count--;
                    xSemaphoreGive(s_buffer_mutex);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));  // 1ms检查一次，提高发送频率
    }

    /* 关闭连接 */
    if (s_tcp_socket >= 0) {
        close(s_tcp_socket);
        s_tcp_socket = -1;
    }
    s_tcp_connected = false;

    ESP_LOGI(TAG, "TCP上传任务结束");
    vTaskDelete(NULL);
}

esp_err_t tcp_data_upload_init(void)
{
    /* 初始化缓冲区索引 */
    s_write_buf_idx = 0;
    s_write_index = 0;
    s_send_buf_idx = 0;
    s_pending_count = 0;

    /* 创建互斥锁 */
    s_buffer_mutex = xSemaphoreCreateMutex();
    if (s_buffer_mutex == NULL) {
        ESP_LOGE(TAG, "创建互斥锁失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TCP数据上传模块初始化完成");
    ESP_LOGI(TAG, "数据包大小: %d 字节, 缓冲区数量: %d", sizeof(tcp_data_packet_t), BUFFER_COUNT);
    return ESP_OK;
}

esp_err_t tcp_data_add_record(const motor_data_record_t *record)
{
    if (record == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 检查模块是否已初始化（后台初始化可能尚未完成） */
    if (s_buffer_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;  /* 模块未初始化，静默丢弃数据 */
    }

    if (xSemaphoreTake(s_buffer_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* 添加数据到当前写缓冲区 */
    memcpy(&s_buffers[s_write_buf_idx][s_write_index], record, sizeof(motor_data_record_t));
    s_write_index++;

    /* 当前缓冲区满，切换到下一个 */
    if (s_write_index >= BUFFER_SIZE) {
        s_write_index = 0;
        s_pending_count++;

        /* 移动到下一个写缓冲区 */
        int next_write_idx = (s_write_buf_idx + 1) % BUFFER_COUNT;

        /* 检查是否会覆盖还未发送的缓冲区 */
        if (s_pending_count >= BUFFER_COUNT) {
            /* 缓冲区队列已满，丢弃最旧的数据 */
            s_pending_count = BUFFER_COUNT - 1;
            s_send_buf_idx = (s_send_buf_idx + 1) % BUFFER_COUNT;
            ESP_LOGW(TAG, "缓冲区队列已满，丢弃最旧数据");
        }

        s_write_buf_idx = next_write_idx;
    }

    xSemaphoreGive(s_buffer_mutex);
    return ESP_OK;
}

esp_err_t tcp_data_upload_start(void)
{
    if (s_upload_running) {
        return ESP_OK;
    }

    s_upload_running = true;

    /* 创建上传任务 */
    BaseType_t ret = xTaskCreate(tcp_upload_task,
                                  "tcp_upload",
                                  4096,
                                  NULL,
                                  5,
                                  &s_upload_task_handle);
    if (ret != pdPASS) {
        s_upload_running = false;
        ESP_LOGE(TAG, "创建上传任务失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TCP数据上传已启动");
    return ESP_OK;
}

void tcp_data_upload_stop(void)
{
    s_upload_running = false;
    ESP_LOGI(TAG, "TCP数据上传已停止");
}

bool tcp_data_is_connected(void)
{
    return s_tcp_connected;
}

void tcp_data_get_stats(uint32_t *packets_sent, uint32_t *packets_failed, uint32_t *bytes_sent)
{
    if (packets_sent) *packets_sent = s_packets_sent;
    if (packets_failed) *packets_failed = s_packets_failed;
    if (bytes_sent) *bytes_sent = s_bytes_sent;
}
