#include "tcp_data_upload.h"
#include "tcp_replay_receive.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_rom_uart.h"
#include "driver/uart.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stdio.h>

/* 串口透传波特率配置 */
#define SERIAL_PASSTHROUGH_BAUDRATE 921600

// static const char *TAG = "TCP_UPLOAD";  // 运行时日志已禁用，TAG未使用

/* 网络配置（由 tcp_set_network_config 初始化） */
static const char *s_wifi_ssid = NULL;
static const char *s_wifi_password = NULL;
static const char *s_tcp_server_host = NULL;
static uint16_t s_tcp_server_port = 0;
static uint16_t s_tcp_lan_server_port = 0;
static uint8_t s_device_id = 0;

/**
 * @brief 设置网络配置
 */
void tcp_set_network_config(const tcp_network_config_t *config)
{
    if (config == NULL) return;
    s_wifi_ssid = config->wifi_ssid;
    s_wifi_password = config->wifi_password;
    s_tcp_server_host = config->tcp_server_host;
    s_tcp_server_port = config->tcp_server_port;
    s_tcp_lan_server_port = config->tcp_lan_server_port;
    s_device_id = config->device_id;
}

/* WiFi事件标志 */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;

/* TCP连接状态 - 公网 */
static int s_tcp_socket = -1;
static bool s_tcp_connected = false;
static bool s_upload_running = false;
static TaskHandle_t s_upload_task_handle = NULL;

/* TCP接收缓冲区（用于接收回放数据） */
static uint8_t s_tcp_rx_buffer[2048];
static int s_tcp_rx_len = 0;

#if defined(ENABLE_WIFI_LAN_TCP)
/* TCP连接状态 - 局域网 */
static int s_tcp_lan_socket = -1;
static bool s_tcp_lan_connected = false;
static esp_ip4_addr_t s_gateway_ip = {0};  /* 网关IP（手机热点地址） */
#endif

#if defined(ENABLE_SERIAL_4G_TCP)
/* 串口透传状态 */
static bool s_serial_initialized = false;
static uint32_t s_serial_packets_sent = 0;
static uint32_t s_serial_bytes_sent = 0;

/* 串口时间同步状态 */
static volatile bool s_serial_ntp_synced = false;     /* 是否已通过串口同步时间 */
// static uint8_t s_serial_rx_buffer[64];                /* 串口接收缓冲区 - 未使用 */
// static volatile int s_serial_rx_len = 0;              /* 接收缓冲区数据长度 - 未使用 */
#endif

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

/* 时间同步状态 */
static volatile bool s_time_synced = false;

/* TCP对时协议定义（与串口透传相同） */
#define TIME_SYNC_REQUEST_BYTE1   0xAA
#define TIME_SYNC_REQUEST_BYTE2   0xCC
#define TIME_SYNC_RESPONSE_BYTE1  0xCC
#define TIME_SYNC_RESPONSE_BYTE2  0xAA
#define TIME_SYNC_CONFIRM_BYTE1   0xBB
#define TIME_SYNC_CONFIRM_BYTE2   0xBB
#define TIME_SYNC_RESPONSE_SIZE   10    /* CC AA + 8字节时间戳 */

/* 前向声明 */
static void clear_data_buffers(void);
static esp_err_t tcp_time_sync(void);
static void tcp_check_receive(void);
static uint8_t calc_crc8(const uint8_t *data, size_t len);
#if defined(ENABLE_SERIAL_4G_TCP)
/* 串口透传前向声明 */
#endif

/* WiFi事件处理 - 仅设置标志，不阻塞 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_tcp_connected = false;
        s_retry_num++;
        /* 频率控制：每5秒打印一次（已禁用以减少串口输出） */
        // int64_t now = esp_timer_get_time() / 1000;
        // if (now - s_last_wifi_log_time >= LOG_INTERVAL_MS) {
        //     ESP_LOGI(TAG, "WiFi重连中 (第%d次)", s_retry_num);
        //     s_last_wifi_log_time = now;
        // }
        esp_wifi_connect();  // 自动重连
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        // ESP_LOGI(TAG, "获取到IP地址: " IPSTR, IP2STR(&event->ip_info.ip));  // 运行时日志已禁用
        (void)event;  /* 避免未使用警告 */
#if defined(ENABLE_WIFI_LAN_TCP)
        /* 保存网关地址（手机热点的IP） */
        s_gateway_ip = event->ip_info.gw;
        // ESP_LOGI(TAG, "网关地址(局域网服务器): " IPSTR, IP2STR(&s_gateway_ip));  // 运行时日志已禁用
#endif
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
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    /* 复制SSID和密码 */
    if (s_wifi_ssid) {
        strncpy((char *)wifi_config.sta.ssid, s_wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
    }
    if (s_wifi_password) {
        strncpy((char *)wifi_config.sta.password, s_wifi_password, sizeof(wifi_config.sta.password) - 1);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // ESP_LOGI(TAG, "WiFi STA初始化完成，正在连接到 %s ...", WIFI_STA_SSID);  // 运行时日志已禁用

    /* 等待连接成功或失败 - 使用超时而非无限等待 */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(10000));  // 10秒超时

    if (bits & WIFI_CONNECTED_BIT) {
        // ESP_LOGI(TAG, "WiFi连接成功: SSID=%s", WIFI_STA_SSID);  // 运行时日志已禁用
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        // ESP_LOGE(TAG, "WiFi连接失败: SSID=%s", WIFI_STA_SSID);  // 运行时日志已禁用
        return ESP_FAIL;
    }

    // ESP_LOGW(TAG, "WiFi连接超时，将在后台继续尝试");  // 运行时日志已禁用
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief 应用服务器时间到系统时钟（公用函数）
 * @param server_time_ms 服务器时间戳（毫秒）
 * @return true 成功, false 时间戳无效
 */
static bool apply_server_time(int64_t server_time_ms)
{
    /* 验证服务器时间戳有效性（2020年~2100年范围内） */
    const int64_t MIN_VALID_TIME_MS = 1577836800000LL;  /* 2020-01-01 */
    const int64_t MAX_VALID_TIME_MS = 4102444800000LL;  /* 2100-01-01 */
    if (server_time_ms < MIN_VALID_TIME_MS || server_time_ms > MAX_VALID_TIME_MS) {
        return false;
    }

    /* 设置时区 */
    setenv("TZ", BEIJING_TIMEZONE, 1);
    tzset();

    /* 设置系统时钟 */
    struct timeval tv;
    tv.tv_sec = server_time_ms / 1000;
    tv.tv_usec = (server_time_ms % 1000) * 1000;
    settimeofday(&tv, NULL);

    /* 设置时间同步标志 */
    s_time_synced = true;
    return true;
}

/**
 * @brief TCP三次握手对时（通过公网TCP连接）
 * @return ESP_OK 成功, ESP_FAIL 失败
 * @note 对时流程:
 *   1. 设备发送 AA CC (对时请求)
 *   2. 服务端回复 CC AA + 8字节时间戳 (对时响应)
 *   3. 设备发送 BB BB (对时确认)
 */
static esp_err_t tcp_time_sync(void)
{
    if (!s_tcp_connected || s_tcp_socket < 0) {
        return ESP_FAIL;
    }

    /* 第一步: 发送对时请求 AA CC */
    uint8_t request_packet[2] = {TIME_SYNC_REQUEST_BYTE1, TIME_SYNC_REQUEST_BYTE2};
    int sent = send(s_tcp_socket, request_packet, 2, 0);
    if (sent != 2) {
        return ESP_FAIL;
    }

    /* 第二步: 等待服务端响应 CC AA + 时间戳，最多等待500ms */
    uint8_t rx_buf[16];
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 500000;  /* 500ms */
    setsockopt(s_tcp_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    int read_len = recv(s_tcp_socket, rx_buf, sizeof(rx_buf), 0);
    if (read_len >= TIME_SYNC_RESPONSE_SIZE) {
        /* 查找 CC AA + 时间戳 响应 */
        for (int i = 0; i <= read_len - TIME_SYNC_RESPONSE_SIZE; i++) {
            if (rx_buf[i] == TIME_SYNC_RESPONSE_BYTE1 && rx_buf[i+1] == TIME_SYNC_RESPONSE_BYTE2) {
                /* 解析8字节时间戳（小端序） */
                int64_t server_time_ms = (int64_t)rx_buf[i+2] |
                                          ((int64_t)rx_buf[i+3] << 8) |
                                          ((int64_t)rx_buf[i+4] << 16) |
                                          ((int64_t)rx_buf[i+5] << 24) |
                                          ((int64_t)rx_buf[i+6] << 32) |
                                          ((int64_t)rx_buf[i+7] << 40) |
                                          ((int64_t)rx_buf[i+8] << 48) |
                                          ((int64_t)rx_buf[i+9] << 56);

                /* 第三步: 收到对时响应，应用时间并回复 BB BB 确认 */
                if (apply_server_time(server_time_ms)) {
                    uint8_t confirm_packet[2] = {TIME_SYNC_CONFIRM_BYTE1, TIME_SYNC_CONFIRM_BYTE2};
                    send(s_tcp_socket, confirm_packet, 2, 0);
                    return ESP_OK;
                }
            }
        }
    }

    return ESP_FAIL;
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
    // ESP_LOGI(TAG, "WiFi后台初始化任务启动");  // 运行时日志已禁用

    /* 初始化WiFi（NTP会在获取到IP后由事件处理器自动启动） */
    esp_err_t wifi_ret = wifi_init_sta();

    /* 初始化TCP上传模块 */
    if (tcp_data_upload_init() == ESP_OK) {
        /* 启动TCP上传任务 */
        tcp_data_upload_start();

        if (wifi_ret == ESP_OK) {
            // ESP_LOGI(TAG, "WiFi和TCP数据上传初始化完成");  // 运行时日志已禁用
        } else {
            // ESP_LOGW(TAG, "WiFi连接未成功，TCP上传任务已启动，将自动重连");  // 运行时日志已禁用
        }
    } else {
        // ESP_LOGE(TAG, "TCP上传模块初始化失败");  // 运行时日志已禁用
    }

    vTaskDelete(NULL);
}

void wifi_tcp_init_background(void)
{
    /* 创建低优先级的后台初始化任务 */
    xTaskCreate(wifi_background_init_task, "wifi_bg_init", 4096, NULL, 2, NULL);
    // ESP_LOGI(TAG, "WiFi/TCP后台初始化任务已创建");  // 运行时日志已禁用
}

/* TCP连接到服务器 */
static esp_err_t tcp_connect_to_server(void)
{
    struct hostent *hp;
    struct sockaddr_in server_addr;
    // int64_t now = esp_timer_get_time() / 1000;  // 日志已禁用，变量未使用

    /* 关闭旧连接 */
    if (s_tcp_socket >= 0) {
        close(s_tcp_socket);
        s_tcp_socket = -1;
    }

    /* DNS解析 */
    hp = gethostbyname(s_tcp_server_host);
    if (hp == NULL) {
        // 运行时日志已禁用以减少串口输出
        // if (now - s_last_tcp_log_time >= LOG_INTERVAL_MS) {
        //     ESP_LOGW(TAG, "TCP: DNS解析失败");
        //     s_last_tcp_log_time = now;
        // }
        return ESP_FAIL;
    }

    /* 创建Socket */
    s_tcp_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s_tcp_socket < 0) {
        // 运行时日志已禁用以减少串口输出
        // if (now - s_last_tcp_log_time >= LOG_INTERVAL_MS) {
        //     ESP_LOGW(TAG, "TCP: Socket创建失败");
        //     s_last_tcp_log_time = now;
        // }
        return ESP_FAIL;
    }

    /* 设置服务器地址 */
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(s_tcp_server_port);
    memcpy(&server_addr.sin_addr, hp->h_addr, hp->h_length);

    /* 连接服务器 */
    if (connect(s_tcp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        // 运行时日志已禁用以减少串口输出
        // if (now - s_last_tcp_log_time >= LOG_INTERVAL_MS) {
        //     ESP_LOGW(TAG, "TCP: 连接服务器失败");
        //     s_last_tcp_log_time = now;
        // }
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
    // ESP_LOGI(TAG, "TCP公网服务器连接成功");  // 运行时日志已禁用

    /* 设置接收超时为非阻塞检查 */
    struct timeval rx_timeout;
    rx_timeout.tv_sec = 0;
    rx_timeout.tv_usec = 1000;  /* 1ms */
    setsockopt(s_tcp_socket, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout));

    return ESP_OK;
}

/**
 * @brief 检查TCP接收数据（非阻塞）
 * @note 处理服务器发来的回放数据
 */
static void tcp_check_receive(void)
{
    if (!s_tcp_connected || s_tcp_socket < 0) {
        return;
    }

    /* 非阻塞接收 */
    int recv_len = recv(s_tcp_socket, s_tcp_rx_buffer + s_tcp_rx_len,
                        sizeof(s_tcp_rx_buffer) - s_tcp_rx_len, MSG_DONTWAIT);

    if (recv_len > 0) {
        s_tcp_rx_len += recv_len;

        /* 处理接收到的数据 */
        int offset = 0;
        while (offset < s_tcp_rx_len) {
            int processed = tcp_replay_process_data(s_tcp_rx_buffer + offset,
                                                     s_tcp_rx_len - offset);
            if (processed > 0) {
                offset += processed;
            } else if (processed == 0) {
                /* 不是回放数据包，跳过一个字节继续查找 */
                offset++;
            } else {
                /* 数据不完整，等待更多数据 */
                break;
            }
        }

        /* 移动剩余数据到缓冲区头部 */
        if (offset > 0 && offset < s_tcp_rx_len) {
            memmove(s_tcp_rx_buffer, s_tcp_rx_buffer + offset, s_tcp_rx_len - offset);
            s_tcp_rx_len -= offset;
        } else if (offset >= s_tcp_rx_len) {
            s_tcp_rx_len = 0;
        }

        /* 检查是否有待发送的参数响应 */
        if (tcp_replay_has_pending_response()) {
            uint8_t response_buf[128];
            int response_len = tcp_replay_get_pending_response(response_buf, sizeof(response_buf));
            if (response_len > 0) {
                /* 填充device_id后需要重新计算CRC */
                response_buf[2] = s_device_id;
                /* 重新计算CRC8（CRC是最后一个字节） */
                response_buf[response_len - 1] = calc_crc8(response_buf, response_len - 1);
                send(s_tcp_socket, response_buf, response_len, 0);
            }
        }
    } else if (recv_len == 0) {
        /* 连接关闭 */
        s_tcp_connected = false;
    }
    /* recv_len < 0 且 errno == EAGAIN/EWOULDBLOCK 表示无数据，正常 */
}

/**
 * @brief 检查并发送回放数据请求（当缓冲区数据不足时）
 */
static void tcp_check_send_replay_request(void)
{
    if (!s_tcp_connected || s_tcp_socket < 0) {
        return;
    }

    /* 检查是否需要请求更多数据 */
    if (!tcp_replay_need_more_data()) {
        return;
    }

    /* 构建请求包 */
    uint8_t request_buf[16];
    int request_len = tcp_replay_build_request_packet(request_buf, sizeof(request_buf));
    if (request_len <= 0) {
        return;
    }

    /* 填充device_id */
    request_buf[2] = s_device_id;

    /* 发送请求 */
    send(s_tcp_socket, request_buf, request_len, 0);
}

#if defined(ENABLE_WIFI_LAN_TCP)
/* TCP连接到局域网服务器（网关/手机热点） */
static esp_err_t tcp_connect_to_lan_server(void)
{
    struct sockaddr_in server_addr;
    // int64_t now = esp_timer_get_time() / 1000;  // 日志已禁用，变量未使用

    /* 检查网关地址是否有效 */
    if (s_gateway_ip.addr == 0) {
        return ESP_FAIL;
    }

    /* 关闭旧连接 */
    if (s_tcp_lan_socket >= 0) {
        close(s_tcp_lan_socket);
        s_tcp_lan_socket = -1;
    }

    /* 创建Socket */
    s_tcp_lan_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s_tcp_lan_socket < 0) {
        // 运行时日志已禁用以减少串口输出
        // if (now - s_last_tcp_log_time >= LOG_INTERVAL_MS) {
        //     ESP_LOGW(TAG, "LAN: Socket创建失败");
        //     s_last_tcp_log_time = now;
        // }
        return ESP_FAIL;
    }

    /* 设置服务器地址（网关IP） */
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(s_tcp_lan_server_port);
    server_addr.sin_addr.s_addr = s_gateway_ip.addr;

    /* 连接服务器 */
    if (connect(s_tcp_lan_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        // 运行时日志已禁用以减少串口输出
        // if (now - s_last_tcp_log_time >= LOG_INTERVAL_MS) {
        //     ESP_LOGW(TAG, "LAN: 连接 " IPSTR ":%d 失败", IP2STR(&s_gateway_ip), s_tcp_lan_server_port);
        //     s_last_tcp_log_time = now;
        // }
        close(s_tcp_lan_socket);
        s_tcp_lan_socket = -1;
        return ESP_FAIL;
    }

    /* 设置发送超时 */
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(s_tcp_lan_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    /* 禁用Nagle算法 */
    int flag = 1;
    setsockopt(s_tcp_lan_socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    s_tcp_lan_connected = true;
    // ESP_LOGI(TAG, "LAN服务器连接成功: " IPSTR ":%d", IP2STR(&s_gateway_ip), s_tcp_lan_server_port);  // 运行时日志已禁用
    return ESP_OK;
}
#endif

/* CRC8查表法（多项式0x07，初始值0x00） */
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

/**
 * @brief 计算CRC8校验值
 * @param data 数据指针
 * @param len 数据长度
 * @return CRC8校验值
 */
static uint8_t calc_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

/* 发送数据包 - 协议v10: 使用静态缓冲区，支持每条数据精确时间偏移和可选roll */
static tcp_packet_header_t s_packet_header;  /* 包头缓冲区 */
static uint8_t s_send_buffer[TCP_PACKET_SIZE_WITH_ROLL];  /* 完整发送缓冲区 */
static size_t s_actual_packet_size = 0;  /* 实际发送的包大小 */

/* 采样间隔（由外部设置，默认5ms） */
static volatile uint8_t s_sample_interval_ms = 5;

/**
 * @brief 设置采样间隔（供main.cpp调用）
 */
void tcp_data_set_interval(uint8_t interval_ms)
{
    s_sample_interval_ms = interval_ms;
}

/* 填充发送数据包（公网和局域网共用） - 协议v10: 每条数据带真实时间偏移 */
static void fill_send_packet(void)
{
    motor_data_record_t *send_buf = s_buffers[s_send_buf_idx];

    /* 检查是否有有效的roll数据（任意一条数据的roll非NAN即有roll） */
    bool has_roll = false;
    for (int i = 0; i < TCP_SAMPLES_PER_PACKET; i++) {
        if (!isnan(send_buf[i].roll_left) || !isnan(send_buf[i].roll_right)) {
            has_roll = true;
            break;
        }
    }

    /* 设置flags: bit0=sync, bit1=has_roll */
    uint8_t flags = 0;
    /* 只要任一同步方式成功即可（WiFi TCP或串口透传） */
    if (s_time_synced) flags |= FLAG_SYNC;
#if defined(ENABLE_SERIAL_4G_TCP)
    if (s_serial_ntp_synced) flags |= FLAG_SYNC;
#endif
    if (has_roll) flags |= FLAG_HAS_ROLL;

    /* 获取基准时间戳（第一条数据的时间，48bit毫秒） */
    int64_t base_time_ms = send_buf[0].timestamp_ms;

    /* 填充包头 */
    s_packet_header.magic = 0xAA55;
    s_packet_header.device_id = s_device_id;
    s_packet_header.version = 10;  /* 协议版本10: 精确时间戳版 */
    s_packet_header.seq = (uint8_t)(s_seq_number++ & 0xFF);  /* 0-255循环 */
    s_packet_header.flags = flags;
    s_packet_header.interval_ms = s_sample_interval_ms;
    s_packet_header.reserved = 0;
    s_packet_header.base_time[0] = (uint8_t)(base_time_ms & 0xFF);
    s_packet_header.base_time[1] = (uint8_t)((base_time_ms >> 8) & 0xFF);
    s_packet_header.base_time[2] = (uint8_t)((base_time_ms >> 16) & 0xFF);
    s_packet_header.base_time[3] = (uint8_t)((base_time_ms >> 24) & 0xFF);
    s_packet_header.base_time[4] = (uint8_t)((base_time_ms >> 32) & 0xFF);
    s_packet_header.base_time[5] = (uint8_t)((base_time_ms >> 40) & 0xFF);

    /* 复制包头到发送缓冲区 */
    size_t offset = 0;
    memcpy(s_send_buffer, &s_packet_header, sizeof(tcp_packet_header_t));
    offset = sizeof(tcp_packet_header_t);

    /* 填充采样数据 */
    if (has_roll) {
        /* 带roll数据: 每条19字节 */
        for (int i = 0; i < TCP_SAMPLES_PER_PACKET; i++) {
            tcp_sample_with_roll_t sample;

            /* 计算相对于第一条数据的时间偏移（毫秒） */
            int64_t time_diff = send_buf[i].timestamp_ms - base_time_ms;
            if (time_diff < 0) time_diff = 0;
            if (time_diff > 65535) time_diff = 65535;
            sample.time_offset_ms = (uint16_t)time_diff;

            /* 电机数据 ×100 转int16，精度0.01 */
            sample.pos1 = (int16_t)(send_buf[i].motor1_pos * 100.0f);
            sample.pos2 = (int16_t)(send_buf[i].motor2_pos * 100.0f);
            sample.vel1 = (int16_t)(send_buf[i].motor1_vel * 100.0f);
            sample.vel2 = (int16_t)(send_buf[i].motor2_vel * 100.0f);
            sample.torque1 = (int16_t)(send_buf[i].motor1_torque * 100.0f);
            sample.torque2 = (int16_t)(send_buf[i].motor2_torque * 100.0f);

            /* roll数据: NAN表示未连接，用特殊值标记 */
            sample.roll_left = isnan(send_buf[i].roll_left) ?
                BT_IMU_NOT_CONNECTED : (int16_t)(send_buf[i].roll_left * 100.0f);
            sample.roll_right = isnan(send_buf[i].roll_right) ?
                BT_IMU_NOT_CONNECTED : (int16_t)(send_buf[i].roll_right * 100.0f);

            /* 状态标签合并: 高4位m1_state, 低4位m2_state */
            sample.states = ((send_buf[i].m1_state_label & 0x0F) << 4) |
                            (send_buf[i].m2_state_label & 0x0F);

            memcpy(s_send_buffer + offset, &sample, sizeof(tcp_sample_with_roll_t));
            offset += sizeof(tcp_sample_with_roll_t);
        }
    } else {
        /* 不带roll数据: 每条15字节 */
        for (int i = 0; i < TCP_SAMPLES_PER_PACKET; i++) {
            tcp_sample_t sample;

            /* 计算相对于第一条数据的时间偏移（毫秒） */
            int64_t time_diff = send_buf[i].timestamp_ms - base_time_ms;
            if (time_diff < 0) time_diff = 0;
            if (time_diff > 65535) time_diff = 65535;
            sample.time_offset_ms = (uint16_t)time_diff;

            /* 电机数据 ×100 转int16，精度0.01 */
            sample.pos1 = (int16_t)(send_buf[i].motor1_pos * 100.0f);
            sample.pos2 = (int16_t)(send_buf[i].motor2_pos * 100.0f);
            sample.vel1 = (int16_t)(send_buf[i].motor1_vel * 100.0f);
            sample.vel2 = (int16_t)(send_buf[i].motor2_vel * 100.0f);
            sample.torque1 = (int16_t)(send_buf[i].motor1_torque * 100.0f);
            sample.torque2 = (int16_t)(send_buf[i].motor2_torque * 100.0f);

            /* 状态标签合并: 高4位m1_state, 低4位m2_state */
            sample.states = ((send_buf[i].m1_state_label & 0x0F) << 4) |
                            (send_buf[i].m2_state_label & 0x0F);

            memcpy(s_send_buffer + offset, &sample, sizeof(tcp_sample_t));
            offset += sizeof(tcp_sample_t);
        }
    }

    /* 计算CRC8校验（从magic到最后一条数据，不包括crc8本身） */
    uint8_t crc8 = calc_crc8(s_send_buffer, offset);
    s_send_buffer[offset] = crc8;
    offset += 1;

    s_actual_packet_size = offset;
}

/* 发送到公网服务器 */
static esp_err_t tcp_send_packet_wan(void)
{
    if (!s_tcp_connected || s_tcp_socket < 0) {
        return ESP_FAIL;
    }

    int sent = send(s_tcp_socket, s_send_buffer, s_actual_packet_size, 0);
    if (sent < 0) {
        // ESP_LOGE(TAG, "公网发送失败: %d", errno);  // 运行时日志已禁用
        s_tcp_connected = false;
        s_packets_failed++;
        return ESP_FAIL;
    }

    s_packets_sent++;
    s_bytes_sent += sent;
    return ESP_OK;
}

#if defined(ENABLE_WIFI_LAN_TCP)
/* 发送到局域网服务器 */
static esp_err_t tcp_send_packet_lan(void)
{
    if (!s_tcp_lan_connected || s_tcp_lan_socket < 0) {
        return ESP_FAIL;
    }

    int sent = send(s_tcp_lan_socket, s_send_buffer, s_actual_packet_size, 0);
    if (sent < 0) {
        // ESP_LOGE(TAG, "局域网发送失败: %d", errno);  // 运行时日志已禁用
        s_tcp_lan_connected = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}
#endif

/**
 * @brief 清空数据发送缓冲区（丢弃旧数据）
 */
static void clear_data_buffers(void)
{
    if (s_buffer_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_write_buf_idx = 0;
        s_write_index = 0;
        s_send_buf_idx = 0;
        s_pending_count = 0;
        memset(s_buffers, 0, sizeof(s_buffers));
        xSemaphoreGive(s_buffer_mutex);
    }
}

#if defined(ENABLE_SERIAL_4G_TCP)
/* 串口透传专用任务句柄 */
static TaskHandle_t s_serial_task_handle = NULL;
static volatile bool s_serial_packet_ready = false;
static uint8_t s_serial_send_buffer[TCP_PACKET_SIZE_WITH_ROLL];  /* 串口发送专用缓冲区 */
static size_t s_serial_packet_size = 0;  /* 串口发送的包大小 */

/* 对时状态 */
static volatile bool s_serial_time_synced = false;  /* 是否已通过串口对时成功 */

/**
 * @brief 检查串口接收的对时响应并设置系统时间
 * @param server_time_ms 输出参数，服务器时间戳（毫秒）
 * @return true 收到对时响应, false 未收到
 */
static bool check_time_sync_response(int64_t *server_time_ms)
{
    /* 检查UART0是否有数据可读 */
    size_t buffered_len = 0;
    uart_get_buffered_data_len(UART_NUM_0, &buffered_len);

    if (buffered_len >= TIME_SYNC_RESPONSE_SIZE) {
        uint8_t rx_buf[16];
        /* 使用较长的超时确保数据完整接收 */
        int read_len = uart_read_bytes(UART_NUM_0, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(20));

        if (read_len >= TIME_SYNC_RESPONSE_SIZE) {
            /* 查找 CC AA + 时间戳 响应 */
            for (int i = 0; i <= read_len - TIME_SYNC_RESPONSE_SIZE; i++) {
                if (rx_buf[i] == TIME_SYNC_RESPONSE_BYTE1 && rx_buf[i+1] == TIME_SYNC_RESPONSE_BYTE2) {
                    /* 解析8字节时间戳（小端序） */
                    *server_time_ms = (int64_t)rx_buf[i+2] |
                                      ((int64_t)rx_buf[i+3] << 8) |
                                      ((int64_t)rx_buf[i+4] << 16) |
                                      ((int64_t)rx_buf[i+5] << 24) |
                                      ((int64_t)rx_buf[i+6] << 32) |
                                      ((int64_t)rx_buf[i+7] << 40) |
                                      ((int64_t)rx_buf[i+8] << 48) |
                                      ((int64_t)rx_buf[i+9] << 56);
                    return true;
                }
            }
        }
    }

    return false;
}

/**
 * @brief 发送对时请求 (AA CC)
 */
static void send_time_sync_request(void)
{
    uint8_t request_packet[2] = {TIME_SYNC_REQUEST_BYTE1, TIME_SYNC_REQUEST_BYTE2};
    int written = uart_write_bytes(UART_NUM_0, request_packet, 2);
    if (written == 2) {
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief 发送对时确认 (BB BB) - 三次握手第三步
 */
static void send_time_sync_confirm(void)
{
    uint8_t confirm_packet[2] = {TIME_SYNC_CONFIRM_BYTE1, TIME_SYNC_CONFIRM_BYTE2};
    int written = uart_write_bytes(UART_NUM_0, confirm_packet, 2);
    if (written == 2) {
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief 串口透传发送任务（独立任务，避免阻塞主上传任务）
 */
static void serial_passthrough_task(void *pvParameters)
{
    uint32_t sync_attempt = 0;
    int64_t server_time_ms = 0;

    /* ========== 阶段1: 三次握手对时（无限循环直到成功） ========== */
    /* 对时流程:
     * 1. 设备发送 AA CC (对时请求)
     * 2. 服务端回复 CC AA + 8字节时间戳 (对时响应)
     * 3. 设备发送 BB BB (对时确认)
     */
    while (!s_serial_time_synced) {
        sync_attempt++;

        /* 清空接收缓冲区 */
        uart_flush_input(UART_NUM_0);

        /* 第一步: 发送对时请求 AA CC */
        send_time_sync_request();

        /* 第二步: 等待服务端响应 CC AA + 时间戳，最多等待500ms */
        for (int i = 0; i < 50; i++) {  /* 50 * 10ms = 500ms */
            vTaskDelay(pdMS_TO_TICKS(10));

            if (check_time_sync_response(&server_time_ms)) {
                /* 第三步: 收到对时响应，应用时间并回复 BB BB 确认 */
                apply_server_time(server_time_ms);
                send_time_sync_confirm();

                s_serial_time_synced = true;
                s_serial_ntp_synced = true;  /* 用于 fill_send_packet 中设置 FLAG_SYNC */
                break;
            }
        }

        if (!s_serial_time_synced) {
            vTaskDelay(pdMS_TO_TICKS(500));  /* 等待后重试 */
        }
    }

    /* 对时成功后清空缓冲区 */
    uart_flush_input(UART_NUM_0);
    clear_data_buffers();

    /* ========== 阶段2: 正常数据传输 ========== */
    while (1) {
        /* 已注册，发送正常数据包 */
        if (s_serial_packet_ready) {
            /* 使用uart_write_bytes确保完全发送 */
            int written = uart_write_bytes(UART_NUM_0, s_serial_send_buffer, s_serial_packet_size);
            if (written == (int)s_serial_packet_size) {
                /* 等待UART TX FIFO发送完毕 */
                uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
            }

            s_serial_packet_ready = false;
            s_serial_packets_sent++;
            s_serial_bytes_sent += s_serial_packet_size;
        }

        /* 丢弃任何意外收到的数据（避免干扰） */
        int len = 0;
        uart_get_buffered_data_len(UART_NUM_0, (size_t *)&len);
        if (len > 0) {
            uart_flush_input(UART_NUM_0);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief 初始化串口透传
 */
static esp_err_t serial_passthrough_init(void)
{
    /* ESP32-S3 UART0默认引脚: TX=GPIO43, RX=GPIO44 */
    const int UART0_TX_PIN = 43;
    const int UART0_RX_PIN = 44;

    /* 配置UART0用于串口透传 */
    uart_config_t uart_config = {
        .baud_rate = SERIAL_PASSTHROUGH_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* 先卸载已有的UART驱动（ESP-IDF控制台可能已安装） */
    if (uart_is_driver_installed(UART_NUM_0)) {
        uart_driver_delete(UART_NUM_0);
    }

    /* 安装UART驱动（必须先安装驱动再配置参数）
     * RX缓冲区设为512字节（足够接收响应），TX缓冲区设为2048字节 */
    esp_err_t err = uart_driver_install(UART_NUM_0, 512, 2048, 0, NULL, 0);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    /* 配置UART参数 */
    uart_param_config(UART_NUM_0, &uart_config);

    /* 显式设置UART0引脚（确保RX引脚正确配置） */
    uart_set_pin(UART_NUM_0, UART0_TX_PIN, UART0_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    /* 等待一小段时间让UART稳定 */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 清空接收缓冲区（丢弃控制台残留数据） */
    uart_flush_input(UART_NUM_0);

    /* 创建独立的串口发送任务，优先级较低，避免阻塞其他任务 */
    BaseType_t ret = xTaskCreate(serial_passthrough_task,
                                  "serial_tx",
                                  4096,
                                  NULL,
                                  3,  /* 低优先级 */
                                  &s_serial_task_handle);
    if (ret != pdPASS) {
        // ESP_LOGE(TAG, "串口透传任务创建失败");  // 运行时日志已禁用
        return ESP_FAIL;
    }

    s_serial_initialized = true;
    // ESP_LOGI(TAG, "串口透传已启用 (UART0, %d bps)", SERIAL_PASSTHROUGH_BAUDRATE);  // 运行时日志已禁用
    return ESP_OK;
}

/**
 * @brief 请求发送数据包（非阻塞，复制数据到发送缓冲区）
 */
static esp_err_t serial_send_packet(void)
{
    if (!s_serial_initialized) {
        return ESP_FAIL;
    }

    /* 如果上一个包还没发完，跳过（避免阻塞） */
    if (s_serial_packet_ready) {
        return ESP_FAIL;
    }

    /* 复制数据包到串口发送缓冲区 */
    memcpy(s_serial_send_buffer, s_send_buffer, s_actual_packet_size);
    s_serial_packet_size = s_actual_packet_size;
    s_serial_packet_ready = true;

    return ESP_OK;
}
#endif

/* 上传任务 */
static void tcp_upload_task(void *pvParameters)
{
    // ESP_LOGI(TAG, "TCP上传任务启动");  // 运行时日志已禁用

#if defined(ENABLE_SERIAL_4G_TCP)
    /* 初始化串口透传 */
    serial_passthrough_init();
#endif

    while (s_upload_running) {
        /* 检查公网TCP连接 */
        if (!s_tcp_connected) {
            /* 静默重连，日志频率已在tcp_connect_to_server中控制 */
            if (tcp_connect_to_server() == ESP_OK) {
                /* 连接成功后进行三次握手对时（如果尚未同步） */
                if (!s_time_synced) {
                    tcp_time_sync();
                    /* 对时成功后清空旧数据 */
                    if (s_time_synced) {
                        clear_data_buffers();
                    }
                }
            }
        }

#if defined(ENABLE_WIFI_LAN_TCP)
        /* 检查局域网TCP连接 */
        if (!s_tcp_lan_connected) {
            tcp_connect_to_lan_server();
        }
#endif

        /* 检查TCP接收数据（回放数据） */
        tcp_check_receive();

        /* 检查是否需要请求更多回放数据 */
        tcp_check_send_replay_request();

        /* 检查是否有数据待发送 */
        if (s_pending_count > 0) {
            /* 填充数据包（公网、局域网、串口透传共用同一个包） */
            fill_send_packet();

            /* 发送到公网 */
            bool wan_ok = (tcp_send_packet_wan() == ESP_OK);

#if defined(ENABLE_WIFI_LAN_TCP)
            /* 发送到局域网 */
            bool lan_ok = (tcp_send_packet_lan() == ESP_OK);
            (void)lan_ok;  /* 避免未使用警告 */
#endif

#if defined(ENABLE_SERIAL_4G_TCP)
            /* 通过串口透传发送（4G模块） - 始终发送，不受WiFi/TCP状态影响 */
            bool serial_ok = (serial_send_packet() == ESP_OK);
            (void)serial_ok;  /* 避免未使用警告 */
#endif

            /* 只要有一个通道发送成功就移动缓冲区（避免数据丢失） */
            bool any_ok = wan_ok;
#if defined(ENABLE_WIFI_LAN_TCP)
            any_ok = any_ok || lan_ok;
#endif
#if defined(ENABLE_SERIAL_4G_TCP)
            any_ok = any_ok || serial_ok;
#endif
            if (any_ok) {
                if (xSemaphoreTake(s_buffer_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    s_send_buf_idx = (s_send_buf_idx + 1) % BUFFER_COUNT;
                    s_pending_count--;
                    xSemaphoreGive(s_buffer_mutex);
                }
            }
        }

        /* 如果所有网络连接都断开，延迟重连 */
        bool need_slow_loop = !s_tcp_connected;
#if defined(ENABLE_WIFI_LAN_TCP)
        need_slow_loop = need_slow_loop && !s_tcp_lan_connected;
#endif
#if defined(ENABLE_SERIAL_4G_TCP)
        /* 串口透传始终工作，不需要慢循环 */
        need_slow_loop = false;
#endif
        if (need_slow_loop) {
            vTaskDelay(pdMS_TO_TICKS(3000));  // 重连间隔3秒
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));  // 1ms检查一次
        }
    }

    /* 关闭连接 */
    if (s_tcp_socket >= 0) {
        close(s_tcp_socket);
        s_tcp_socket = -1;
    }
    s_tcp_connected = false;

#if defined(ENABLE_WIFI_LAN_TCP)
    if (s_tcp_lan_socket >= 0) {
        close(s_tcp_lan_socket);
        s_tcp_lan_socket = -1;
    }
    s_tcp_lan_connected = false;
#endif

    // ESP_LOGI(TAG, "TCP上传任务结束");  // 运行时日志已禁用
    vTaskDelete(NULL);
}

esp_err_t tcp_data_upload_init(void)
{
    /* 初始化缓冲区索引 */
    s_write_buf_idx = 0;
    s_write_index = 0;
    s_send_buf_idx = 0;
    s_pending_count = 0;
    s_tcp_rx_len = 0;

    /* 创建互斥锁 */
    s_buffer_mutex = xSemaphoreCreateMutex();
    if (s_buffer_mutex == NULL) {
        // ESP_LOGE(TAG, "创建互斥锁失败");  // 运行时日志已禁用
        return ESP_FAIL;
    }

    /* 初始化回放模块 */
    tcp_replay_init();

    // ESP_LOGI(TAG, "TCP数据上传模块初始化完成");  // 运行时日志已禁用
    // ESP_LOGI(TAG, "数据包大小: %d 字节, 缓冲区数量: %d", sizeof(tcp_data_packet_t), BUFFER_COUNT);  // 运行时日志已禁用
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
            /* 频率控制：每5秒打印一次（已禁用以减少串口输出） */
            // int64_t now = esp_timer_get_time() / 1000;
            // if (now - s_last_buffer_log_time >= LOG_INTERVAL_MS) {
            //     ESP_LOGW(TAG, "缓冲区满，丢弃旧数据");
            //     s_last_buffer_log_time = now;
            // }
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
        // ESP_LOGE(TAG, "创建上传任务失败");  // 运行时日志已禁用
        return ESP_FAIL;
    }

    // ESP_LOGI(TAG, "TCP数据上传已启动");  // 运行时日志已禁用
    return ESP_OK;
}

void tcp_data_upload_stop(void)
{
    s_upload_running = false;
    // ESP_LOGI(TAG, "TCP数据上传已停止");  // 运行时日志已禁用
}

bool tcp_data_is_connected(void)
{
    return s_tcp_connected;
}

#if defined(ENABLE_WIFI_LAN_TCP)
bool tcp_data_lan_is_connected(void)
{
    return s_tcp_lan_connected;
}

bool tcp_get_gateway_ip(char *ip_str)
{
    if (ip_str == NULL || s_gateway_ip.addr == 0) {
        return false;
    }
    sprintf(ip_str, IPSTR, IP2STR(&s_gateway_ip));
    return true;
}
#else
bool tcp_data_lan_is_connected(void)
{
    return false;
}

bool tcp_get_gateway_ip(char *ip_str)
{
    return false;
}
#endif

void tcp_data_get_stats(uint32_t *packets_sent, uint32_t *packets_failed, uint32_t *bytes_sent)
{
    if (packets_sent) *packets_sent = s_packets_sent;
    if (packets_failed) *packets_failed = s_packets_failed;
    if (bytes_sent) *bytes_sent = s_bytes_sent;
}
