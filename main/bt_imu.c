/**
 * @file bt_imu.c
 * @brief BT IMU 连接器模块实现
 */

#include "bt_imu.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "esp_central.h"

static const char *TAG = "BT_IMU";

// 支持多个IMU设备
#define MAX_IMU_DEVICES 2

// 目标IMU设备地址列表
// 格式: {低字节, ..., 高字节}
static const uint8_t target_addrs[MAX_IMU_DEVICES][6] = {
    {0x37, 0x0a, 0x16, 0xbf, 0x0c, 0x00},  // 设备1: 00:0c:bf:16:0a:37 左
    {0x4f, 0x74, 0x06, 0xbf, 0x0c, 0x00}   // 设备2: 00:0c:bf:06:74:4f 右
};

// IMU设备UUID常量（基于参考Python代码）
static const ble_uuid128_t imu_service_uuid =
    BLE_UUID128_INIT(0x55, 0xe4, 0x05, 0xd2, 0xaf, 0x9f, 0xa9, 0x8f,
                     0xe5, 0x4a, 0x7d, 0xfe, 0x43, 0x53, 0x53, 0x49);

static const ble_uuid128_t imu_read_char_uuid =
    BLE_UUID128_INIT(0x16, 0x96, 0x24, 0x47, 0xc6, 0x23, 0x61, 0xba,
                     0xd9, 0x4b, 0x4d, 0x1e, 0x43, 0x53, 0x53, 0x49);

static const ble_uuid128_t imu_write_char_uuid =
    BLE_UUID128_INIT(0xb3, 0x9b, 0x72, 0x34, 0xbe, 0xec, 0xd4, 0xa8,
                     0xf4, 0x43, 0x41, 0x88, 0x43, 0x53, 0x53, 0x49);

// BT IMU句柄结构
struct bt_imu_handle {
    // BLE连接状态
    bool is_connected;
    uint16_t conn_handle;
    uint16_t write_char_handle;
    uint16_t read_char_handle;

    // 设备标识
    uint8_t device_addr[6];  // 设备MAC地址
    int device_index;         // 设备索引(0或1)

    // 数据缓存和保护
    bt_imu_data_t sensor_data;
    SemaphoreHandle_t data_mutex;

    // 统计信息
    uint32_t total_bytes_received;
    uint32_t packet_count;
    uint32_t checksum_errors;  // 校验和错误计数
    uint32_t sync_resets;      // 重新同步计数

    // 临时数据缓存（11字节数据包）
    uint8_t temp_bytes[11];
    int temp_bytes_len;

    // 初始化标志
    bool initialized;
};

// 全局句柄数组（支持同时连接多个设备）
static bt_imu_handle_t* g_bt_imu_handles[MAX_IMU_DEVICES] = {NULL, NULL};
static int g_connected_count = 0;  // 已连接设备数量
static bool g_connecting_in_progress = false;  // 是否有连接操作正在进行
static bool g_need_rescan = false;  // 是否需要重新扫描

// 兼容旧接口的全局句柄（指向第一个设备）
static bt_imu_handle_t* g_bt_imu_handle = NULL;

// 前置声明
static void bt_imu_scan(void);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static int find_target_device_index(const ble_addr_t *addr);
static bt_imu_handle_t* find_handle_by_conn(uint16_t conn_handle);
static bool is_device_connected(int device_index);
void ble_store_config_init(void);

/**
 * 格式化地址字符串
 */
static char* format_addr_str(const uint8_t *addr) {
    static char addr_str_buf[18];
    snprintf(addr_str_buf, sizeof(addr_str_buf), "%02x:%02x:%02x:%02x:%02x:%02x",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    return addr_str_buf;
}

/**
 * 获取有符号16位数
 */
static int16_t get_sign_int16(uint16_t num) {
    if (num >= 32768) {
        return num - 65536;
    }
    return num;
}

/**
 * 处理IMU数据包（11字节格式）
 */
static void process_imu_data(bt_imu_handle_t* handle, const uint8_t *data) {
    if (!handle || data[0] != 0x55) {
        return;
    }

    // 校验和检查
    uint8_t checksum = 0;
    for (int i = 0; i < 10; i++) {
        checksum += data[i];
    }
    if ((checksum & 0xff) != data[10]) {
        // 校验失败，尝试在缓冲区中重新寻找0x55同步头
        handle->checksum_errors++;
        ESP_LOGD(TAG, "校验和错误: 计算=%02x 实际=%02x [总计:%lu]",
                 checksum & 0xff, data[10], handle->checksum_errors);

        // 在缓冲区中查找下一个可能的0x55起始位置
        for (int i = 1; i < 11; i++) {
            if (data[i] == 0x55) {
                // 找到新的同步头，将剩余数据移到缓冲区开头
                int remaining = 11 - i;
                memmove((void*)handle->temp_bytes, &data[i], remaining);
                handle->temp_bytes_len = remaining;
                handle->sync_resets++;
                ESP_LOGD(TAG, "重新同步: 偏移=%d 剩余=%d [总计:%lu]",
                         i, remaining, handle->sync_resets);
                return;
            }
        }

        // 没找到新的同步头，清空缓冲区
        handle->temp_bytes_len = 0;
        return;
    }

    bt_imu_data_t temp_data = handle->sensor_data; // 保持之前的数据

    // 根据数据类型解析
    switch (data[1]) {
        case 0x50: // 时间 Time
        {
            int year = data[2] + 2000;
            int mon = data[3];
            int day = data[4];
            int hour = data[5];
            int minute = data[6];
            int sec = data[7];
            int mils = (data[9] << 8) | data[8];
            snprintf(temp_data.time_str, sizeof(temp_data.time_str),
                     "%04d-%02d-%02d %02d:%02d:%02d:%03d",
                     year, mon, day, hour, minute, sec, mils);
            break;
        }

        case 0x51: // 加速度 Acceleration
        {
            int16_t ax_raw = (data[3] << 8) | data[2];
            int16_t ay_raw = (data[5] << 8) | data[4];
            int16_t az_raw = (data[7] << 8) | data[6];

            temp_data.acc_x = get_sign_int16(ax_raw) / 32768.0f * 16.0f;
            temp_data.acc_y = get_sign_int16(ay_raw) / 32768.0f * 16.0f;
            temp_data.acc_z = get_sign_int16(az_raw) / 32768.0f * 16.0f;
            temp_data.is_valid = true;
            break;
        }

        case 0x52: // 角速度 Angular velocity
        {
            int16_t gx_raw = (data[3] << 8) | data[2];
            int16_t gy_raw = (data[5] << 8) | data[4];
            int16_t gz_raw = (data[7] << 8) | data[6];

            temp_data.gyro_x = get_sign_int16(gx_raw) / 32768.0f * 2000.0f;
            temp_data.gyro_y = get_sign_int16(gy_raw) / 32768.0f * 2000.0f;
            temp_data.gyro_z = get_sign_int16(gz_raw) / 32768.0f * 2000.0f;
            break;
        }

        case 0x53: // 角度 Angle
        {
            int16_t angx_raw = (data[3] << 8) | data[2];
            int16_t angy_raw = (data[5] << 8) | data[4];
            int16_t angz_raw = (data[7] << 8) | data[6];

            // 映射到roll, pitch, yaw
            temp_data.roll = get_sign_int16(angx_raw) / 32768.0f * 180.0f;   // AngleX -> Roll
            temp_data.pitch = get_sign_int16(angy_raw) / 32768.0f * 180.0f;  // AngleY -> Pitch
            temp_data.yaw = get_sign_int16(angz_raw) / 32768.0f * 180.0f;    // AngleZ -> Yaw
            temp_data.is_valid = true;
            break;
        }

        case 0x54: // 磁场 Magnetic field
        {
            int16_t hx_raw = (data[3] << 8) | data[2];
            int16_t hy_raw = (data[5] << 8) | data[4];
            int16_t hz_raw = (data[7] << 8) | data[6];

            temp_data.mag_x = get_sign_int16(hx_raw) / 120.0f;
            temp_data.mag_y = get_sign_int16(hy_raw) / 120.0f;
            temp_data.mag_z = get_sign_int16(hz_raw) / 120.0f;
            break;
        }

        default:
            ESP_LOGD(TAG, "未知数据类型: 0x%02x", data[1]);
            return; // 未知数据类型
    }

    temp_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // 线程安全更新数据
    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        handle->sensor_data = temp_data;
        handle->sensor_data.packet_count++;
        handle->total_bytes_received += 11;
        xSemaphoreGive(handle->data_mutex);
    }
}

/**
 * 数据接收回调
 */
static int on_data_received(uint16_t conn_handle,
                           const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr,
                           void *arg) {
    if (error->status != 0) {
        return 0;
    }

    // 查找对应的设备句柄
    bt_imu_handle_t* handle = find_handle_by_conn(conn_handle);
    if (!handle) {
        ESP_LOGW(TAG, "未找到连接句柄: %d", conn_handle);
        return 0;
    }

    // 将数据复制到临时缓存
    uint8_t data[attr->om->om_len];
    ble_hs_mbuf_to_flat(attr->om, data, attr->om->om_len, NULL);

    // 逐字节处理数据包
    for (int i = 0; i < attr->om->om_len; i++) {
        // 防止缓冲区溢出
        if (handle->temp_bytes_len >= 11) {
            ESP_LOGW(TAG, "设备#%d缓冲区溢出，重置", handle->device_index + 1);
            handle->temp_bytes_len = 0;
        }

        handle->temp_bytes[handle->temp_bytes_len++] = data[i];

        // 检查数据包头
        if (handle->temp_bytes_len == 1 && handle->temp_bytes[0] != 0x55) {
            handle->temp_bytes_len = 0;
            continue;
        }

        // 处理完整数据包（11字节）
        if (handle->temp_bytes_len == 11) {
            process_imu_data(handle, handle->temp_bytes);
            handle->temp_bytes_len = 0;
        }
    }

    return 0;
}

/**
 * 服务发现完成回调
 */
static void on_service_discovery_complete(const struct peer *peer, int status, void *arg) {
    const struct peer_svc *svc;
    const struct peer_chr *chr;

    // 从参数中获取conn_handle
    uint16_t conn_handle = (uint16_t)(uintptr_t)arg;
    bt_imu_handle_t* handle = find_handle_by_conn(conn_handle);

    if (!handle || status != 0) {
        ESP_LOGE(TAG, "服务发现失败: %d", status);
        if (handle) {
            ble_gap_terminate(handle->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return;
    }

    ESP_LOGI(TAG, "设备#%d服务发现完成", handle->device_index + 1);

    // 查找IMU服务
    svc = peer_svc_find_uuid(peer, &imu_service_uuid.u);
    if (svc == NULL) {
        ESP_LOGE(TAG, "设备#%d未找到IMU服务", handle->device_index + 1);
        ble_gap_terminate(handle->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    ESP_LOGI(TAG, "设备#%d找到IMU服务", handle->device_index + 1);

    // 查找写特征值
    chr = peer_chr_find_uuid(peer, &imu_service_uuid.u, &imu_write_char_uuid.u);
    if (chr != NULL) {
        handle->write_char_handle = chr->chr.val_handle;
        ESP_LOGI(TAG, "设备#%d找到写特征值，句柄: %d", handle->device_index + 1, handle->write_char_handle);
    }

    // 查找读特征值（通知）
    chr = peer_chr_find_uuid(peer, &imu_service_uuid.u, &imu_read_char_uuid.u);
    if (chr != NULL) {
        handle->read_char_handle = chr->chr.val_handle;
        ESP_LOGI(TAG, "设备#%d找到读特征值，句柄: %d", handle->device_index + 1, handle->read_char_handle);

        // 启用通知
        const struct peer_dsc *dsc;
        uint16_t notify_val = 1;
        int rc = -1;

        SLIST_FOREACH(dsc, &chr->dscs, next) {
            if (ble_uuid_cmp(&dsc->dsc.uuid.u, BLE_UUID16_DECLARE(0x2902)) == 0) {
                rc = ble_gattc_write_flat(handle->conn_handle, dsc->dsc.handle,
                                         &notify_val, sizeof(notify_val), NULL, NULL);
                break;
            }
        }

        if (rc != 0) {
            ESP_LOGE(TAG, "设备#%d启用通知失败: %d", handle->device_index + 1, rc);
        } else {
            ESP_LOGI(TAG, "设备#%d通知已启用，等待数据...", handle->device_index + 1);
        }
    }

    if (handle->read_char_handle == 0) {
        ESP_LOGE(TAG, "设备#%d IMU特征值配置失败", handle->device_index + 1);
        ble_gap_terminate(handle->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    // 服务发现完成，检查是否还有未连接的设备
    bool has_unconnected = false;
    for (int i = 0; i < MAX_IMU_DEVICES; i++) {
        if (!is_device_connected(i)) {
            has_unconnected = true;
            break;
        }
    }

    if (has_unconnected) {
        ESP_LOGI(TAG, "设备#%d配置完成，标记需要继续扫描", handle->device_index + 1);
        g_need_rescan = true;
    } else {
        ESP_LOGI(TAG, "所有设备已连接并配置完成");
    }
}

/**
 * 检查设备地址是否是目标设备之一
 * 返回设备索引，如果不是目标设备则返回-1
 */
static int find_target_device_index(const ble_addr_t *addr) {
    for (int i = 0; i < MAX_IMU_DEVICES; i++) {
        if (memcmp(addr->val, target_addrs[i], 6) == 0) {
            return i;
        }
    }
    return -1;
}

/**
 * 根据连接句柄查找对应的IMU句柄
 */
static bt_imu_handle_t* find_handle_by_conn(uint16_t conn_handle) {
    for (int i = 0; i < MAX_IMU_DEVICES; i++) {
        if (g_bt_imu_handles[i] &&
            g_bt_imu_handles[i]->conn_handle == conn_handle) {
            return g_bt_imu_handles[i];
        }
    }
    return NULL;
}

/**
 * 检查设备是否已经连接
 */
static bool is_device_connected(int device_index) {
    return (g_bt_imu_handles[device_index] != NULL &&
            g_bt_imu_handles[device_index]->is_connected);
}

/**
 * GAP事件处理
 */
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
    {
        int device_index = find_target_device_index(&event->disc.addr);
        if (device_index >= 0 && !is_device_connected(device_index)) {
            ESP_LOGI(TAG, "发现目标IMU设备#%d: %s", device_index + 1, format_addr_str(event->disc.addr.val));
            ESP_LOGI(TAG, "信号强度: %d dBm", event->disc.rssi);

            // 检查是否已经有连接操作正在进行
            if (g_connecting_in_progress) {
                ESP_LOGI(TAG, "设备#%d已发现，等待当前连接完成后再连接", device_index + 1);
                return 0;
            }

            // 不要立即取消扫描，允许继续扫描其他设备
            uint8_t own_addr_type;
            rc = ble_hs_id_infer_auto(0, &own_addr_type);
            if (rc != 0) {
                ESP_LOGE(TAG, "地址类型推断失败: %d", rc);
                return 0;
            }

            // 为该设备分配句柄（如果尚未分配）
            if (g_bt_imu_handles[device_index] == NULL) {
                g_bt_imu_handles[device_index] = (bt_imu_handle_t*)malloc(sizeof(bt_imu_handle_t));
                if (!g_bt_imu_handles[device_index]) {
                    ESP_LOGE(TAG, "设备#%d内存分配失败", device_index + 1);
                    return 0;
                }
                memset(g_bt_imu_handles[device_index], 0, sizeof(bt_imu_handle_t));

                // 创建互斥锁
                g_bt_imu_handles[device_index]->data_mutex = xSemaphoreCreateMutex();
                if (!g_bt_imu_handles[device_index]->data_mutex) {
                    ESP_LOGE(TAG, "设备#%d互斥锁创建失败", device_index + 1);
                    free(g_bt_imu_handles[device_index]);
                    g_bt_imu_handles[device_index] = NULL;
                    return 0;
                }

                g_bt_imu_handles[device_index]->initialized = true;
            }

            // 记录设备信息
            memcpy(g_bt_imu_handles[device_index]->device_addr, event->disc.addr.val, 6);
            g_bt_imu_handles[device_index]->device_index = device_index;

            ESP_LOGI(TAG, "正在连接IMU设备#%d...", device_index + 1);

            // 先取消扫描，连接完成后会自动重新扫描未连接的设备
            ble_gap_disc_cancel();

            rc = ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, ble_gap_event, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "设备#%d连接失败: %d", device_index + 1, rc);
                g_connecting_in_progress = false;
                // 标记需要重新扫描
                g_need_rescan = true;
            } else {
                // 标记连接操作正在进行
                g_connecting_in_progress = true;
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
    {
        // 清除连接进行中标志
        g_connecting_in_progress = false;

        // 通过地址找到对应的设备索引
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGE(TAG, "获取连接信息失败: %d", rc);
            // 标记需要重新扫描
            g_need_rescan = true;
            return 0;
        }

        int device_index = find_target_device_index(&desc.peer_id_addr);
        if (device_index < 0 || !g_bt_imu_handles[device_index]) {
            ESP_LOGE(TAG, "未知设备连接");
            // 标记需要重新扫描
            g_need_rescan = true;
            return 0;
        }

        bt_imu_handle_t* handle = g_bt_imu_handles[device_index];

        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "IMU设备#%d连接成功！地址: %s",
                     device_index + 1,
                     format_addr_str(handle->device_addr));

            handle->is_connected = true;
            handle->conn_handle = event->connect.conn_handle;
            g_connected_count++;

            ESP_LOGI(TAG, "连接句柄: %d (已连接%d/%d个设备)",
                     handle->conn_handle, g_connected_count, MAX_IMU_DEVICES);
            ESP_LOGI(TAG, "开始服务发现...");

            rc = peer_add(handle->conn_handle);
            if (rc != 0) {
                ESP_LOGE(TAG, "设备#%d添加peer失败: %d", device_index + 1, rc);
                // 标记需要重新扫描
                g_need_rescan = true;
                return 0;
            }

            // 传递conn_handle给回调函数
            rc = peer_disc_all(handle->conn_handle, on_service_discovery_complete,
                              (void*)(uintptr_t)handle->conn_handle);
            if (rc != 0) {
                ESP_LOGE(TAG, "设备#%d服务发现启动失败: %d", device_index + 1, rc);
                // 标记需要重新扫描
                g_need_rescan = true;
                return 0;
            }

            // 不在这里立即开始扫描，等待服务发现完成后再扫描
            // 服务发现完成后会在回调中继续扫描

        } else {
            ESP_LOGE(TAG, "设备#%d连接失败，状态: %d", device_index + 1, event->connect.status);
            // 标记需要重新扫描
            g_need_rescan = true;
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT:
    {
        // 清除连接进行中标志
        g_connecting_in_progress = false;

        // 查找对应的设备句柄
        bt_imu_handle_t* handle = find_handle_by_conn(event->disconnect.conn.conn_handle);
        if (!handle) {
            ESP_LOGW(TAG, "断开未知设备");
            return 0;
        }

        ESP_LOGI(TAG, "IMU设备#%d已断开连接，原因: %d", handle->device_index + 1, event->disconnect.reason);

        // 只清理该设备的连接状态
        handle->is_connected = false;
        handle->conn_handle = 0;
        handle->write_char_handle = 0;
        handle->read_char_handle = 0;
        g_connected_count--;

        // 清理数据有效性
        if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            handle->sensor_data.is_valid = false;
            xSemaphoreGive(handle->data_mutex);
        }

        peer_delete(event->disconnect.conn.conn_handle);

        ESP_LOGI(TAG, "设备断开，标记重新扫描 (当前已连接%d/%d个设备)", g_connected_count, MAX_IMU_DEVICES);
        g_need_rescan = true;
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
    {
        // 检查是否所有设备都已连接
        bool all_connected = true;
        for (int i = 0; i < MAX_IMU_DEVICES; i++) {
            if (!is_device_connected(i)) {
                all_connected = false;
                break;
            }
        }

        if (!all_connected) {
            ESP_LOGI(TAG, "扫描完成，未找到所有目标设备 (已连接%d/%d)，标记重新扫描",
                     g_connected_count, MAX_IMU_DEVICES);
            g_need_rescan = true;
        } else {
            ESP_LOGI(TAG, "所有目标设备已连接");
        }
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX:
        return on_data_received(event->notify_rx.conn_handle,
                               &(struct ble_gatt_error){.status = 0},
                               &(struct ble_gatt_attr){
                                   .handle = event->notify_rx.attr_handle,
                                   .om = event->notify_rx.om
                               },
                               NULL);

    default:
        return 0;
    }
}

/**
 * 开始扫描IMU设备
 */
static void bt_imu_scan(void) {
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "地址类型确定失败: %d", rc);
        return;
    }

    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.filter_duplicates = 1;
    disc_params.passive = 1;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    ESP_LOGI(TAG, "开始扫描目标IMU设备 (共%d个)", MAX_IMU_DEVICES);
    for (int i = 0; i < MAX_IMU_DEVICES; i++) {
        ESP_LOGI(TAG, "  设备#%d: %s", i + 1, format_addr_str(target_addrs[i]));
    }

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "扫描启动失败: %d", rc);
    }
}

/**
 * 蓝牙主机同步回调
 */
static void ble_on_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "地址设置失败: %d", rc);
        return;
    }

    bt_imu_scan();
}

/**
 * 连接监控任务 - 处理重新扫描
 */
static void connection_monitor_task(void *param) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000)); // 每2秒检查一次

        if (g_need_rescan && !g_connecting_in_progress) {
            g_need_rescan = false;
            ESP_LOGI(TAG, "执行延迟的扫描请求...");
            bt_imu_scan();
        }
    }
}

/**
 * 蓝牙主机重置回调
 */
static void ble_on_reset(int reason) {
    ESP_LOGE(TAG, "蓝牙主机重置: %d", reason);
}

/**
 * 蓝牙主机任务
 */
static void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BT IMU连接器启动");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// =============================================================================
// 公共接口实现
// =============================================================================

bt_imu_handle_t* bt_imu_init(void) {
    if (g_bt_imu_handle != NULL) {
        ESP_LOGW(TAG, "BT IMU已经初始化");
        return g_bt_imu_handle;
    }

    // 分配第一个设备的句柄内存
    g_bt_imu_handles[0] = (bt_imu_handle_t*)malloc(sizeof(bt_imu_handle_t));
    if (!g_bt_imu_handles[0]) {
        ESP_LOGE(TAG, "内存分配失败");
        return NULL;
    }

    // 初始化句柄
    memset(g_bt_imu_handles[0], 0, sizeof(bt_imu_handle_t));

    // 创建互斥锁
    g_bt_imu_handles[0]->data_mutex = xSemaphoreCreateMutex();
    if (!g_bt_imu_handles[0]->data_mutex) {
        ESP_LOGE(TAG, "互斥锁创建失败");
        free(g_bt_imu_handles[0]);
        g_bt_imu_handles[0] = NULL;
        return NULL;
    }

    // 设置兼容指针
    g_bt_imu_handle = g_bt_imu_handles[0];

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS初始化失败: %d", ret);
        vSemaphoreDelete(g_bt_imu_handles[0]->data_mutex);
        free(g_bt_imu_handles[0]);
        g_bt_imu_handles[0] = NULL;
        g_bt_imu_handle = NULL;
        return NULL;
    }

    // 初始化NimBLE
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE初始化失败: %d", ret);
        vSemaphoreDelete(g_bt_imu_handles[0]->data_mutex);
        free(g_bt_imu_handles[0]);
        g_bt_imu_handles[0] = NULL;
        g_bt_imu_handle = NULL;
        return NULL;
    }

    // 配置主机
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // 初始化peer管理
    int rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    if (rc != 0) {
        ESP_LOGE(TAG, "peer初始化失败: %d", rc);
        vSemaphoreDelete(g_bt_imu_handles[0]->data_mutex);
        free(g_bt_imu_handles[0]);
        g_bt_imu_handles[0] = NULL;
        g_bt_imu_handle = NULL;
        return NULL;
    }

    // 设置设备名称
    rc = ble_svc_gap_device_name_set("ESP32S3-Balance");
    if (rc != 0) {
        ESP_LOGE(TAG, "设备名称设置失败: %d", rc);
    }

    // 配置存储
    ble_store_config_init();

    // 启动NimBLE主机任务
    nimble_port_freertos_init(ble_host_task);

    g_bt_imu_handles[0]->initialized = true;
    ESP_LOGI(TAG, "BT IMU模块初始化成功");

    return g_bt_imu_handle;
}

void bt_imu_destroy(bt_imu_handle_t* handle) {
    if (handle != g_bt_imu_handle || !handle) {
        return;
    }

    // 断开连接
    if (handle->is_connected) {
        ble_gap_terminate(handle->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }

    // 销毁互斥锁
    if (handle->data_mutex) {
        vSemaphoreDelete(handle->data_mutex);
    }

    // 释放内存
    free(handle);
    g_bt_imu_handle = NULL;

    ESP_LOGI(TAG, "BT IMU模块已销毁");
}

bool bt_imu_get_data(bt_imu_handle_t* handle, bt_imu_data_t* data) {
    if (!handle || !data || !handle->initialized) {
        return false;
    }

    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *data = handle->sensor_data;
        xSemaphoreGive(handle->data_mutex);
        return handle->sensor_data.is_valid;
    }

    return false;
}

float bt_imu_get_roll(bt_imu_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return 0.0f;
    }

    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        float roll = handle->sensor_data.roll;
        xSemaphoreGive(handle->data_mutex);
        return roll;
    }

    return 0.0f;
}

float bt_imu_get_pitch(bt_imu_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return 0.0f;
    }

    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        float pitch = handle->sensor_data.pitch;
        xSemaphoreGive(handle->data_mutex);
        return pitch;
    }

    return 0.0f;
}

float bt_imu_get_yaw(bt_imu_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return 0.0f;
    }

    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        float yaw = handle->sensor_data.yaw;
        xSemaphoreGive(handle->data_mutex);
        return yaw;
    }

    return 0.0f;
}

bool bt_imu_is_connected(bt_imu_handle_t* handle) {
    return handle && handle->initialized && handle->is_connected;
}

uint32_t bt_imu_get_bytes_received(bt_imu_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return 0;
    }

    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        uint32_t bytes = handle->total_bytes_received;
        xSemaphoreGive(handle->data_mutex);
        return bytes;
    }

    return 0;
}

uint32_t bt_imu_get_checksum_errors(bt_imu_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return 0;
    }

    return handle->checksum_errors;
}

// =============================================================================
// 多设备API实现
// =============================================================================

int bt_imu_init_multi(void) {
    static bool nimble_initialized = false;

    // 检查是否已经有设备初始化
    for (int i = 0; i < MAX_IMU_DEVICES; i++) {
        if (g_bt_imu_handles[i] != NULL) {
            ESP_LOGW(TAG, "BT IMU多设备已经初始化");
            return 0;
        }
    }

    // 如果没有初始化过NimBLE，则进行初始化
    if (!nimble_initialized) {
        // 初始化NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS初始化失败: %d", ret);
            return -1;
        }

        // 初始化NimBLE
        ret = nimble_port_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "NimBLE初始化失败: %d", ret);
            return -1;
        }

        // 配置主机
        ble_hs_cfg.reset_cb = ble_on_reset;
        ble_hs_cfg.sync_cb = ble_on_sync;
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

        // 初始化peer管理
        int rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
        if (rc != 0) {
            ESP_LOGE(TAG, "peer初始化失败: %d", rc);
            return -1;
        }

        // 设置设备名称
        rc = ble_svc_gap_device_name_set("ESP32S3-Balance");
        if (rc != 0) {
            ESP_LOGE(TAG, "设备名称设置失败: %d", rc);
        }

        // 配置存储
        ble_store_config_init();

        // 启动NimBLE主机任务
        nimble_port_freertos_init(ble_host_task);

        // 启动连接监控任务（增加栈大小）
        xTaskCreate(connection_monitor_task, "bt_imu_monitor", 4096, NULL, 5, NULL);

        nimble_initialized = true;
        ESP_LOGI(TAG, "BT IMU多设备模块初始化成功，准备连接%d个设备", MAX_IMU_DEVICES);

        // 设置兼容指针（当第一个设备连接时会自动设置）
    }

    return 0;
}

bool bt_imu_get_data_multi(int device_index, bt_imu_data_t* data) {
    if (device_index < 0 || device_index >= MAX_IMU_DEVICES || !data) {
        return false;
    }

    bt_imu_handle_t* handle = g_bt_imu_handles[device_index];
    if (!handle || !handle->initialized) {
        return false;
    }

    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *data = handle->sensor_data;
        xSemaphoreGive(handle->data_mutex);
        return handle->sensor_data.is_valid;
    }

    return false;
}

float bt_imu_get_pitch_multi(int device_index) {
    if (device_index < 0 || device_index >= MAX_IMU_DEVICES) {
        return 0.0f;
    }

    bt_imu_handle_t* handle = g_bt_imu_handles[device_index];
    if (!handle || !handle->initialized) {
        return 0.0f;
    }

    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        float pitch = handle->sensor_data.pitch;
        xSemaphoreGive(handle->data_mutex);
        return pitch;
    }

    return 0.0f;
}

bool bt_imu_is_connected_multi(int device_index) {
    if (device_index < 0 || device_index >= MAX_IMU_DEVICES) {
        return false;
    }

    return is_device_connected(device_index);
}

uint32_t bt_imu_get_bytes_received_multi(int device_index) {
    if (device_index < 0 || device_index >= MAX_IMU_DEVICES) {
        return 0;
    }

    bt_imu_handle_t* handle = g_bt_imu_handles[device_index];
    if (!handle || !handle->initialized) {
        return 0;
    }

    if (xSemaphoreTake(handle->data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        uint32_t bytes = handle->total_bytes_received;
        xSemaphoreGive(handle->data_mutex);
        return bytes;
    }

    return 0;
}

uint32_t bt_imu_get_checksum_errors_multi(int device_index) {
    if (device_index < 0 || device_index >= MAX_IMU_DEVICES) {
        return 0;
    }

    bt_imu_handle_t* handle = g_bt_imu_handles[device_index];
    if (!handle || !handle->initialized) {
        return 0;
    }

    return handle->checksum_errors;
}

int bt_imu_get_connected_count(void) {
    return g_connected_count;
}
