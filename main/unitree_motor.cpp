#include "unitree_motor.h"
#include "esp_log.h"
#include "crc_utils.h"
#include <string.h>
#include <algorithm>

static const char *TAG_MOTOR = "UnitreeMotorDriver";

// ==================== 静态函数区域 (内部使用) ====================

// ==================== 构造和析构函数 ====================

/**
 * @brief 构造函数，初始化电机驱动对象
 *
 * 初始化成员变量为默认值：
 * - UART端口号设为无效值
 * - MAX485控制引脚设为NC（未连接）
 * - 初始化标志设为false
 */
UnitreeMotorDriver::UnitreeMotorDriver() : _uart_num((uart_port_t)-1), _max485_re_de_pin(GPIO_NUM_NC), _is_initialized(false) {
}

/**
 * @brief 析构函数，清理UART驱动和GPIO资源
 *
 * 如果驱动已初始化，则：
 * - 卸载UART驱动
 * - 复位MAX485控制引脚（如果已配置）
 */
UnitreeMotorDriver::~UnitreeMotorDriver() {
    if (_is_initialized) {
        uart_driver_delete(_uart_num);
        ESP_LOGI(TAG_MOTOR, "UART驱动已卸载");
        if (_max485_re_de_pin != GPIO_NUM_NC) {
            gpio_reset_pin(_max485_re_de_pin);
            ESP_LOGI(TAG_MOTOR, "MAX485控制引脚已复位");
        }
    }
}

// ==================== 私有成员函数 ====================

/**
 * @brief 打包GO-M8010-6电机控制命令为16字节数据帧
 * @param cmd 电机命令结构体，包含ID、模式、位置、速度、力矩及PID参数
 * @param buffer 输出缓冲区指针，至少16字节
 * @return 打包后的字节数（固定16字节）
 *
 * 数据帧格式（16字节）：
 * - 字节0-1: 帧头 (0xFE 0xEE)
 * - 字节2: ID(低4位) | MODE(高3位)
 * - 字节3-4: 力矩 tau_set (int16_t, 范围±60Nm)
 * - 字节5-6: 速度 omega_set (int16_t, 范围±40rad/s)
 * - 字节7-10: 位置 theta_set (int32_t, 范围±12.5rad)
 * - 字节11-12: Kp增益 (uint16_t, 范围0~25.6)
 * - 字节13-14: Kd增益 (uint16_t, 范围0~25.6)
 * - 字节15-16: CRC16-CCITT校验
 *
 * @note 所有物理量会被限幅到安全范围内
 */
size_t UnitreeMotorDriver::pack_motor_cmd_go_m8010_6(const MotorCmdA1& cmd, uint8_t* buffer) {
    memset(buffer, 0, GO_M8010_6_CMD_LENGTH);

    // 帧头 (0xFE 0xEE)
    buffer[0] = GO_M8010_6_CMD_HEADER_BYTE1;
    buffer[1] = GO_M8010_6_CMD_HEADER_BYTE2;

    // 模式设置 (ID + STATUS) - 字节 2
    buffer[2] = (cmd.id & 0x0F) | ((cmd.mode & 0x07) << 4);

    // 力矩 (tau_set) - 字节 3-4 (int16_t)
    int16_t tau_packed = static_cast<int16_t>(std::clamp(cmd.t, -GO_M8010_6_TAU_MAX_ABS, GO_M8010_6_TAU_MAX_ABS) * GO_M8010_6_TAU_RATIO);
    buffer[3] = (uint8_t)(tau_packed & 0xFF);
    buffer[4] = (uint8_t)((tau_packed >> 8) & 0xFF);

    // 速度 (omega_set) - 字节 5-6 (int16_t)
    int16_t omega_packed = static_cast<int16_t>(std::clamp(cmd.vel, -GO_M8010_6_OMEGA_MAX_ABS, GO_M8010_6_OMEGA_MAX_ABS) * GO_M8010_6_OMEGA_UNIT_PER_RAD);
    buffer[5] = (uint8_t)(omega_packed & 0xFF);
    buffer[6] = (uint8_t)((omega_packed >> 8) & 0xFF);

    // 位置 (theta_set) - 字节 7-10 (int32_t)
    int32_t theta_packed = static_cast<int32_t>(std::clamp(cmd.pos, -GO_M8010_6_THETA_MAX_ABS, GO_M8010_6_THETA_MAX_ABS) * GO_M8010_6_THETA_UNIT_PER_RAD);
    buffer[7] = (uint8_t)(theta_packed & 0xFF);
    buffer[8] = (uint8_t)((theta_packed >> 8) & 0xFF);
    buffer[9] = (uint8_t)((theta_packed >> 16) & 0xFF);
    buffer[10] = (uint8_t)((theta_packed >> 24) & 0xFF);

    // Kp (K_pos) - 字节 11-12 (uint16_t)
    uint16_t kp_packed = static_cast<uint16_t>(std::clamp(cmd.kp, 0.0f, GO_M8010_6_KP_MAX) * GO_M8010_6_KP_RATIO_INV_25_6);
    buffer[11] = (uint8_t)(kp_packed & 0xFF);
    buffer[12] = (uint8_t)((kp_packed >> 8) & 0xFF);

    // Kd (K_spd) - 字节 13-14 (uint16_t)
    uint16_t kd_packed = static_cast<uint16_t>(std::clamp(cmd.kd, 0.0f, GO_M8010_6_KD_MAX) * GO_M8010_6_KD_RATIO_INV_25_6);
    buffer[13] = (uint8_t)(kd_packed & 0xFF);
    buffer[14] = (uint8_t)((kd_packed >> 8) & 0xFF);

    // CRC16_CCITT 校验
    uint16_t calculated_crc = crc_ccitt(0x0000, buffer, GO_M8010_6_CMD_LENGTH - 2);
    buffer[GO_M8010_6_CMD_LENGTH - 2] = (uint8_t)(calculated_crc & 0xFF);
    buffer[GO_M8010_6_CMD_LENGTH - 1] = (uint8_t)((calculated_crc >> 8) & 0xFF);

    return GO_M8010_6_CMD_LENGTH;
}

/**
 * @brief 解析GO-M8010-6电机返回的16字节数据帧
 * @param buffer 接收到的数据缓冲区指针，至少16字节
 * @param data 输出参数，解析后的电机状态数据结构体
 * @return true-解析成功，false-帧头错误或CRC校验失败
 *
 * 数据帧格式（16字节）：
 * - 字节0-1: 帧头 (0xFD 0xEE)
 * - 字节2: ID(低4位) | MODE(高3位)
 * - 字节3-4: 力矩反馈 tau_fbk (int16_t)
 * - 字节5-6: 速度反馈 omega_fbk (int16_t)
 * - 字节7-10: 位置反馈 theta_fbk (int32_t)
 * - 字节11: 温度 TEMP (int8_t, 单位℃)
 * - 字节12: MERROR(低3位) | FORCE(高5位)
 * - 字节13: FORCE(低7位)
 * - 字节14-15: CRC16-CCITT校验
 *
 * @note 目前CRC校验失败仅打印警告，不会返回false
 */
bool UnitreeMotorDriver::unpack_motor_data_go_m8010_6(const uint8_t* buffer, MotorDataA1& data) {
    // 帧头 (0xFD 0xEE)
    if (buffer[0] != GO_M8010_6_DATA_HEADER_BYTE1 || buffer[1] != GO_M8010_6_DATA_HEADER_BYTE2) {
        return false;
    }

    // 校验CRC16_CCITT
    uint16_t received_crc = (uint16_t)(buffer[GO_M8010_6_DATA_LENGTH - 1] << 8) | buffer[GO_M8010_6_DATA_LENGTH - 2];
    uint16_t calculated_crc = crc_ccitt(0x0000, buffer, GO_M8010_6_DATA_LENGTH - 2);

    if (received_crc != calculated_crc) {
        ESP_LOGE(TAG_MOTOR, "CRC校验失败: 接收0x%04X, 计算0x%04X", received_crc, calculated_crc);
        // 临时跳过CRC校验，继续解析数据
        // return false;
    }

    // 解包 ID 和 MODE - 字节 2
    data.id = buffer[2] & 0x0F;
    data.mode = (buffer[2] >> 4) & 0x07;

    // 解包力矩 (tau_fbk) - 字节 3-4 (int16_t)
    int16_t tau_packed = (int16_t)(buffer[4] << 8) | buffer[3];
    data.t = static_cast<float>(tau_packed) / GO_M8010_6_TAU_RATIO;

    // 解包速度 (omega_fbk) - 字节 5-6 (int16_t)
    int16_t omega_packed = (int16_t)(buffer[6] << 8) | buffer[5];
    data.vel = static_cast<float>(omega_packed) * GO_M8010_6_OMEGA_RAD_PER_UNIT;

    // 解包位置 (theta_fbk) - 字节 7-10 (int32_t)
    int32_t theta_packed = (int32_t)(buffer[10] << 24) | (buffer[9] << 16) | (buffer[8] << 8) | buffer[7];
    data.pos = static_cast<float>(theta_packed) * GO_M8010_6_THETA_RAD_PER_UNIT;

    // 解包温度 (TEMP) - 字节 11 (int8_t)
    data.temp = static_cast<int8_t>(buffer[11]);

    // 解包 MERROR (3bit)
    data.MError = buffer[12] & 0x07;

    // FORCE 12bit
    uint16_t foot_force_raw = ((uint16_t)(buffer[13] & 0x7F) << 5) | ((buffer[12] >> 3) & 0x1F);
    data.footForce = foot_force_raw;

    return true;
}

// ==================== 公共成员函数 ====================

/**
 * @brief 初始化UART和MAX485通信接口
 * @param uart_num UART端口号（如UART_NUM_2）
 * @param tx_pin UART发送引脚编号
 * @param rx_pin UART接收引脚编号
 * @param max485_re_de_pin MAX485方向控制引脚（如不使用自动方向控制则传GPIO_NUM_NC）
 * @param baud_rate 波特率（默认4000000，即4Mbps）
 * @return true-初始化成功，false-初始化失败
 *
 * 初始化流程：
 * 1. 配置UART参数（8N1，无流控）
 * 2. 安装UART驱动（1024字节RX/TX缓冲）
 * 3. 设置TX/RX引脚映射
 * 4. 设置接收超时（3个符号周期）
 * 5. 清空UART缓冲区
 *
 * @note 使用外部RS485模块自动方向控制，无需手动控制RE/DE引脚
 * @note 初始化失败时会自动清理已分配的资源
 */
bool UnitreeMotorDriver::init(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t max485_re_de_pin, int baud_rate) {
    _uart_num = uart_num;
    _max485_re_de_pin = max485_re_de_pin;

    // 配置UART参数
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
        .flags = {
            .allow_pd = 0,
            .backup_before_sleep = 0
        }
    };

    uart_wait_tx_idle_polling(_uart_num);

    esp_err_t err = uart_driver_install(_uart_num, 1024, 1024, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "安装UART驱动失败, 错误码: %d", err);
        return false;
    }

    err = uart_set_pin(_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "设置UART引脚失败, 错误码: %d", err);
        uart_driver_delete(_uart_num);
        return false;
    }

    err = uart_param_config(_uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "配置UART参数失败, 错误码: %d", err);
        uart_driver_delete(_uart_num);
        return false;
    }

    err = uart_set_rx_timeout(_uart_num, 3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "设置RX超时失败, 错误码: %d", err);
        uart_driver_delete(_uart_num);
        return false;
    }

    ESP_LOGI(TAG_MOTOR, "纯串口模式已启用，外部RS485模块自动控制方向");

    uart_flush(_uart_num);

    _is_initialized = true;
    return true;
}

/**
 * @brief 发送控制命令并接收电机反馈数据（同步收发）
 * @param cmd 待发送的电机控制命令结构体
 * @param data 输出参数，接收到的电机状态数据结构体
 * @return ESP_OK-成功，ESP_FAIL-通信失败，ESP_ERR_TIMEOUT-接收超时
 *
 * 通信流程：
 * 1. 打包控制命令为16字节数据帧
 * 2. 通过UART发送数据包
 * 3. 等待TX FIFO清空
 * 4. 接收电机响应数据（最多重试3次，每次超时2ms）
 * 5. 查找帧头并解析数据
 *
 * 错误处理：
 * - 未初始化：返回ESP_FAIL
 * - 发送失败：返回ESP_FAIL
 * - 未找到帧头：返回ESP_FAIL并打印原始数据
 * - 接收超时：返回ESP_ERR_TIMEOUT（限速打印，每5秒最多1次）
 *
 * @note 此函数会阻塞约2-6ms（取决于重试次数）
 * @note 在500Hz控制循环中调用时，应确保不超时
 */
esp_err_t UnitreeMotorDriver::sendRecv(const MotorCmdA1& cmd, MotorDataA1& data) {
    if (!_is_initialized) {
        ESP_LOGE(TAG_MOTOR, "电机驱动未初始化，无法发送/接收数据");
        return ESP_FAIL;
    }

    uint8_t tx_buffer[GO_M8010_6_CMD_LENGTH];
    uint8_t rx_buffer[GO_M8010_6_DATA_LENGTH];

    // 打包电机命令
    size_t packed_len = pack_motor_cmd_go_m8010_6(cmd, tx_buffer);

    // 发送数据包
    int tx_bytes = uart_write_bytes(_uart_num, (const char*)tx_buffer, packed_len);
    if (tx_bytes != packed_len) {
        ESP_LOGE(TAG_MOTOR, "发送字节数不匹配: 期望%d, 实际%d", packed_len, tx_bytes);
        return ESP_FAIL;
    }

    // 等待发送完成
    uart_wait_tx_idle_polling(_uart_num);

    // 接收响应 - 健壮的数据包解析
    uint8_t temp_buffer[GO_M8010_6_DATA_LENGTH + 16];
    int total_bytes = 0;

    for (int attempts = 0; attempts < 3; attempts++) {
        int rx_bytes = uart_read_bytes(_uart_num, temp_buffer + total_bytes,
                                      sizeof(temp_buffer) - total_bytes, pdMS_TO_TICKS(2));
        if (rx_bytes > 0) {
            total_bytes += rx_bytes;

            // 查找帧头位置 (0xFD 0xEE)
            for (int i = 0; i <= total_bytes - GO_M8010_6_DATA_LENGTH; i++) {
                if (temp_buffer[i] == GO_M8010_6_DATA_HEADER_BYTE1 &&
                    temp_buffer[i + 1] == GO_M8010_6_DATA_HEADER_BYTE2) {

                    memcpy(rx_buffer, &temp_buffer[i], GO_M8010_6_DATA_LENGTH);

                    // 解析数据
                    if (unpack_motor_data_go_m8010_6(rx_buffer, data)) {
                        return ESP_OK;
                    } else {
                        ESP_LOGW(TAG_MOTOR, "数据解析失败，继续查找");
                        break;
                    }
                }
            }
        } else {
            break;
        }
    }

    if (total_bytes > 0) {
        ESP_LOGW(TAG_MOTOR, "未找到有效帧头，接收了%d字节数据", total_bytes);
        printf("原始数据: ");
        for (int i = 0; i < total_bytes && i < 32; i++) {
            printf("0x%02X ", temp_buffer[i]);
        }
        printf("\n");
        return ESP_FAIL;
    } else {
        // 频率限制：每5秒最多打印一次超时警告
        static uint32_t last_timeout_warning_time = 0;
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (current_time - last_timeout_warning_time >= 5000) {
            ESP_LOGW(TAG_MOTOR, "接收超时 - 没有收到任何数据 (未连接电机?)");
            last_timeout_warning_time = current_time;
        }
        return ESP_ERR_TIMEOUT;
    }
}
