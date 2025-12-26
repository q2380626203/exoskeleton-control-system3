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
#include "esp_rom_uart.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stdio.h>

static const char *TAG = "TCP_UPLOAD";

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

#if TCP_ENABLE_LAN_UPLOAD
/* TCP连接状态 - 局域网 */
static int s_tcp_lan_socket = -1;
static bool s_tcp_lan_connected = false;
static esp_ip4_addr_t s_gateway_ip = {0};  /* 网关IP（手机热点地址） */
#endif

#if SERIAL_PASSTHROUGH_ENABLE
/* 串口透传状态 */
static bool s_serial_initialized = false;
static uint32_t s_serial_packets_sent = 0;
static uint32_t s_serial_bytes_sent = 0;
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

/* NTP时间同步状态 */
static volatile bool s_time_synced = false;
static volatile bool s_sntp_initialized = false;  /* SNTP是否已初始化 */

/* 日志频率控制 (5秒间隔) */
#define LOG_INTERVAL_MS 5000
static int64_t s_last_wifi_log_time = 0;
static int64_t s_last_tcp_log_time = 0;
static int64_t s_last_buffer_log_time = 0;

/* 前向声明 */
static void ensure_ntp_running(void);

/* WiFi事件处理 - 仅设置标志，不阻塞 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_tcp_connected = false;
        s_time_synced = false;  /* WiFi断开时标记时间未同步 */
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
#if TCP_ENABLE_LAN_UPLOAD
        /* 保存网关地址（手机热点的IP） */
        s_gateway_ip = event->ip_info.gw;
        // ESP_LOGI(TAG, "网关地址(局域网服务器): " IPSTR, IP2STR(&s_gateway_ip));  // 运行时日志已禁用
#endif
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        /* 网络连接后确保NTP运行 */
        ensure_ntp_running();
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

/* NTP时间同步回调 */
static void time_sync_notification_cb(struct timeval *tv)
{
    s_time_synced = true;

    time_t now = tv->tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    // ESP_LOGI(TAG, "NTP时间同步成功: %04d-%02d-%02d %02d:%02d:%02d (北京时间)",
    //          timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
    //          timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);  // 运行时日志已禁用
}

void ntp_time_sync_init(void)
{
    // ESP_LOGI(TAG, "初始化SNTP时间同步...");  // 运行时日志已禁用

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

    s_sntp_initialized = true;
    // ESP_LOGI(TAG, "SNTP已启动，等待时间同步...");  // 运行时日志已禁用
}

/**
 * @brief 确保NTP服务正在运行（用于WiFi重连后）
 * @note 如果SNTP未初始化则初始化，如果已初始化则重启同步
 */
static void ensure_ntp_running(void)
{
    if (!s_sntp_initialized) {
        /* 首次初始化 */
        ntp_time_sync_init();
    } else {
        /* 已初始化，重启SNTP以触发重新同步 */
        // ESP_LOGI(TAG, "WiFi重连，重启NTP时间同步...");  // 运行时日志已禁用
        esp_sntp_restart();
    }
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
    int64_t now = esp_timer_get_time() / 1000;

    /* 关闭旧连接 */
    if (s_tcp_socket >= 0) {
        close(s_tcp_socket);
        s_tcp_socket = -1;
    }

    /* DNS解析 */
    hp = gethostbyname(TCP_SERVER_HOST);
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
    server_addr.sin_port = htons(TCP_SERVER_PORT);
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
    return ESP_OK;
}

#if TCP_ENABLE_LAN_UPLOAD
/* TCP连接到局域网服务器（网关/手机热点） */
static esp_err_t tcp_connect_to_lan_server(void)
{
    struct sockaddr_in server_addr;
    int64_t now = esp_timer_get_time() / 1000;

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
    server_addr.sin_port = htons(TCP_LAN_SERVER_PORT);
    server_addr.sin_addr.s_addr = s_gateway_ip.addr;

    /* 连接服务器 */
    if (connect(s_tcp_lan_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        // 运行时日志已禁用以减少串口输出
        // if (now - s_last_tcp_log_time >= LOG_INTERVAL_MS) {
        //     ESP_LOGW(TAG, "LAN: 连接 " IPSTR ":%d 失败", IP2STR(&s_gateway_ip), TCP_LAN_SERVER_PORT);
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
    // ESP_LOGI(TAG, "LAN服务器连接成功: " IPSTR ":%d", IP2STR(&s_gateway_ip), TCP_LAN_SERVER_PORT);  // 运行时日志已禁用
    return ESP_OK;
}
#endif

/* 发送数据包 - 使用静态缓冲区避免栈溢出 */
static tcp_data_packet_t s_send_packet;  /* 静态发送缓冲区 */

/* 填充发送数据包（公网和局域网共用） */
static void fill_send_packet(void)
{
    s_send_packet.magic = 0xAA55;
    s_send_packet.device_id = DEVICE_ID;
    s_send_packet.version = 6;  /* 协议版本6: 添加蓝牙IMU roll数据 */
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
        /* 蓝牙IMU roll数据: NAN表示未连接，用0x7FFF标记 */
        s_send_packet.samples[i].roll_left = isnan(send_buf[i].roll_left) ?
            BT_IMU_NOT_CONNECTED : (int16_t)(send_buf[i].roll_left * 100.0f);
        s_send_packet.samples[i].roll_right = isnan(send_buf[i].roll_right) ?
            BT_IMU_NOT_CONNECTED : (int16_t)(send_buf[i].roll_right * 100.0f);
        s_send_packet.samples[i].m1_state = send_buf[i].m1_state_label;
        s_send_packet.samples[i].m2_state = send_buf[i].m2_state_label;
    }
}

/* 发送到公网服务器 */
static esp_err_t tcp_send_packet_wan(void)
{
    if (!s_tcp_connected || s_tcp_socket < 0) {
        return ESP_FAIL;
    }

    int sent = send(s_tcp_socket, &s_send_packet, sizeof(s_send_packet), 0);
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

#if TCP_ENABLE_LAN_UPLOAD
/* 发送到局域网服务器 */
static esp_err_t tcp_send_packet_lan(void)
{
    if (!s_tcp_lan_connected || s_tcp_lan_socket < 0) {
        return ESP_FAIL;
    }

    int sent = send(s_tcp_lan_socket, &s_send_packet, sizeof(s_send_packet), 0);
    if (sent < 0) {
        // ESP_LOGE(TAG, "局域网发送失败: %d", errno);  // 运行时日志已禁用
        s_tcp_lan_connected = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}
#endif

#if SERIAL_PASSTHROUGH_ENABLE
/* 串口透传专用任务句柄 */
static TaskHandle_t s_serial_task_handle = NULL;
static volatile bool s_serial_packet_ready = false;
static tcp_data_packet_t s_serial_send_packet;  /* 串口发送专用缓冲区 */

/**
 * @brief 串口透传发送任务（独立任务，避免阻塞主上传任务）
 */
static void serial_passthrough_task(void *pvParameters)
{
    // ESP_LOGI(TAG, "串口透传任务启动 (UART0)");  // 运行时日志已禁用

    while (1) {
        /* 等待数据包就绪 */
        if (s_serial_packet_ready) {
            /* 使用fwrite写入stdout，阻塞等待发送完成 */
            fwrite(&s_serial_send_packet, 1, sizeof(s_serial_send_packet), stdout);
            fflush(stdout);

            s_serial_packet_ready = false;
            s_serial_packets_sent++;
            s_serial_bytes_sent += sizeof(s_serial_send_packet);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief 初始化串口透传
 */
static esp_err_t serial_passthrough_init(void)
{
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
    // ESP_LOGI(TAG, "串口透传已启用 (UART0, 独立任务)");  // 运行时日志已禁用
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
    memcpy(&s_serial_send_packet, &s_send_packet, sizeof(tcp_data_packet_t));
    s_serial_packet_ready = true;

    return ESP_OK;
}
#endif

/* 上传任务 */
static void tcp_upload_task(void *pvParameters)
{
    // ESP_LOGI(TAG, "TCP上传任务启动");  // 运行时日志已禁用

#if SERIAL_PASSTHROUGH_ENABLE
    /* 初始化串口透传 */
    serial_passthrough_init();
#endif

    while (s_upload_running) {
        /* 检查公网TCP连接 */
        if (!s_tcp_connected) {
            /* 静默重连，日志频率已在tcp_connect_to_server中控制 */
            tcp_connect_to_server();
        }

#if TCP_ENABLE_LAN_UPLOAD
        /* 检查局域网TCP连接 */
        if (!s_tcp_lan_connected) {
            tcp_connect_to_lan_server();
        }
#endif

        /* 检查是否有数据待发送 */
        if (s_pending_count > 0) {
            /* 填充数据包（公网、局域网、串口透传共用同一个包） */
            fill_send_packet();

            /* 发送到公网 */
            bool wan_ok = (tcp_send_packet_wan() == ESP_OK);

#if TCP_ENABLE_LAN_UPLOAD
            /* 发送到局域网 */
            bool lan_ok = (tcp_send_packet_lan() == ESP_OK);
            (void)lan_ok;  /* 避免未使用警告 */
#endif

#if SERIAL_PASSTHROUGH_ENABLE
            /* 通过串口透传发送（4G模块） - 始终发送，不受WiFi/TCP状态影响 */
            bool serial_ok = (serial_send_packet() == ESP_OK);
            (void)serial_ok;  /* 避免未使用警告 */
#endif

            /* 只要有一个通道发送成功就移动缓冲区（避免数据丢失） */
            bool any_ok = wan_ok;
#if TCP_ENABLE_LAN_UPLOAD
            any_ok = any_ok || lan_ok;
#endif
#if SERIAL_PASSTHROUGH_ENABLE
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
#if TCP_ENABLE_LAN_UPLOAD
        need_slow_loop = need_slow_loop && !s_tcp_lan_connected;
#endif
#if SERIAL_PASSTHROUGH_ENABLE
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

#if TCP_ENABLE_LAN_UPLOAD
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

    /* 创建互斥锁 */
    s_buffer_mutex = xSemaphoreCreateMutex();
    if (s_buffer_mutex == NULL) {
        // ESP_LOGE(TAG, "创建互斥锁失败");  // 运行时日志已禁用
        return ESP_FAIL;
    }

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

#if TCP_ENABLE_LAN_UPLOAD
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
