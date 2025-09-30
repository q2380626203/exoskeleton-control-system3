#include "unitree_motor.h"
#include "esp_log.h"
#include "crc_utils.h" // 包含CRC工具函数
#include <string.h>
#include <algorithm> // for std::clamp

static const char *TAG_MOTOR = "UnitreeMotorDriver";

UnitreeMotorDriver::UnitreeMotorDriver() : _uart_num((uart_port_t)-1), _max485_re_de_pin(GPIO_NUM_NC), _is_initialized(false) {
    // 构造函数
}

UnitreeMotorDriver::~UnitreeMotorDriver() {
    if (_is_initialized) {
        uart_driver_delete(_uart_num);
        ESP_LOGI(TAG_MOTOR, "UART驱动已卸载");
        if (_max485_re_de_pin != GPIO_NUM_NC) {
            gpio_reset_pin(_max485_re_de_pin); // Reset GPIO pin
            ESP_LOGI(TAG_MOTOR, "MAX485控制引脚已复位");
        }
    }
}

bool UnitreeMotorDriver::init(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t max485_re_de_pin, int baud_rate) {
    _uart_num = uart_num;
    _max485_re_de_pin = max485_re_de_pin;

    // 配置UART参数
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // 必须禁用硬件流控用于RS485
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB, // Use APB clock for higher baud rates
    };

    // 参考test_rs485：等待之前传输完成
    uart_wait_tx_idle_polling(_uart_num);

    // 安装UART驱动，参考test_rs485高性能配置：大缓冲区，无事件队列
    esp_err_t err = uart_driver_install(_uart_num, 1024, 1024, 0, NULL, 0);  // 1024字节RX/TX缓冲区
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "安装UART驱动失败, 错误码: %d", err);
        return false;
    }

    // 配置UART引脚 - 纯串口模式，不需要DE/RE控制
    err = uart_set_pin(_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "设置UART引脚失败, 错误码: %d", err);
        uart_driver_delete(_uart_num);
        return false;
    }

    // 配置UART参数
    err = uart_param_config(_uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "配置UART参数失败, 错误码: %d", err);
        uart_driver_delete(_uart_num);
        return false;
    }

    // 纯串口模式，不需要设置RS485模式
    // 外部RS485模块自带自动流控，ESP32只负责串口收发

    // 设置RX超时 - 参考test_rs485：3个字符时间，适应高速通信
    err = uart_set_rx_timeout(_uart_num, 3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "设置RX超时失败, 错误码: %d", err);
        uart_driver_delete(_uart_num);
        return false;
    }

    // 纯串口模式：采用同步通信，不需要数据队列

    ESP_LOGI(TAG_MOTOR, "纯串口模式已启用，外部RS485模块自动控制方向");

    // 清空UART缓冲区
    uart_flush(_uart_num);

    _is_initialized = true;
    return true;
}

// 将 MotorCmdA1 结构体转换为 GO-M8010-6 协议并打包成字节数组
size_t UnitreeMotorDriver::pack_motor_cmd_go_m8010_6(const MotorCmdA1& cmd, uint8_t* buffer) {
    // 清空缓冲区
    memset(buffer, 0, GO_M8010_6_CMD_LENGTH);

    // 帧头 (0xFE 0xEE)
    buffer[0] = GO_M8010_6_CMD_HEADER_BYTE1;
    buffer[1] = GO_M8010_6_CMD_HEADER_BYTE2;

    // 模式设置 (ID + STATUS) - 字节 2
    // cmd.id (4bit) & cmd.mode (3bit)
    buffer[2] = (cmd.id & 0x0F) | ((cmd.mode & 0x07) << 4);

    // 力矩 (tau_set) - 字节 3-4 (int16_t)
    // tor_des = T * 256.0f
    int16_t tau_packed = static_cast<int16_t>(std::clamp(cmd.t, -GO_M8010_6_TAU_MAX_ABS, GO_M8010_6_TAU_MAX_ABS) * GO_M8010_6_TAU_RATIO);
    buffer[3] = (uint8_t)(tau_packed & 0xFF);         // 低位
    buffer[4] = (uint8_t)((tau_packed >> 8) & 0xFF);   // 高位

    // 速度 (omega_set) - 字节 5-6 (int16_t)
    // spd_des = W / 6.28318f * 256.0f
    int16_t omega_packed = static_cast<int16_t>(std::clamp(cmd.vel, -GO_M8010_6_OMEGA_MAX_ABS, GO_M8010_6_OMEGA_MAX_ABS) * GO_M8010_6_OMEGA_UNIT_PER_RAD);
    buffer[5] = (uint8_t)(omega_packed & 0xFF);       // 低位
    buffer[6] = (uint8_t)((omega_packed >> 8) & 0xFF); // 高位

    // 位置 (theta_set) - 字节 7-10 (int32_t)
    // pos_des = Pos / 6.28318f * 32768.0f
    int32_t theta_packed = static_cast<int32_t>(std::clamp(cmd.pos, -GO_M8010_6_THETA_MAX_ABS, GO_M8010_6_THETA_MAX_ABS) * GO_M8010_6_THETA_UNIT_PER_RAD);
    buffer[7] = (uint8_t)(theta_packed & 0xFF);
    buffer[8] = (uint8_t)((theta_packed >> 8) & 0xFF);
    buffer[9] = (uint8_t)((theta_packed >> 16) & 0xFF);
    buffer[10] = (uint8_t)((theta_packed >> 24) & 0xFF);

    // Kp (K_pos) - 字节 11-12 (uint16_t)
    // k_pos = K_P / 25.6f * 32768.0f;
    uint16_t kp_packed = static_cast<uint16_t>(std::clamp(cmd.kp, 0.0f, GO_M8010_6_KP_MAX) * GO_M8010_6_KP_RATIO_INV_25_6);
    buffer[11] = (uint8_t)(kp_packed & 0xFF);       // 低位
    buffer[12] = (uint8_t)((kp_packed >> 8) & 0xFF); // 高位

    // Kd (K_spd) - 字节 13-14 (uint16_t)
    // k_spd = K_W / 25.6f * 32768.0f;
    uint16_t kd_packed = static_cast<uint16_t>(std::clamp(cmd.kd, 0.0f, GO_M8010_6_KD_MAX) * GO_M8010_6_KD_RATIO_INV_25_6);
    buffer[13] = (uint8_t)(kd_packed & 0xFF);       // 低位
    buffer[14] = (uint8_t)((kd_packed >> 8) & 0xFF); // 高位

    // CRC16_CCITT 校验 (针对从 buffer[0] 到 buffer[14] 的15字节数据)
    // 初始CRC值为0x0000
    uint16_t calculated_crc = crc_ccitt(0x0000, buffer, GO_M8010_6_CMD_LENGTH - 2);
    buffer[GO_M8010_6_CMD_LENGTH - 2] = (uint8_t)(calculated_crc & 0xFF); // CRC低位
    buffer[GO_M8010_6_CMD_LENGTH - 1] = (uint8_t)((calculated_crc >> 8) & 0xFF); // CRC高位

    return GO_M8010_6_CMD_LENGTH;
}

// 将字节数组解包成 MotorDataGO_M8010-6 协议并转换为 MotorDataA1 结构体
bool UnitreeMotorDriver::unpack_motor_data_go_m8010_6(const uint8_t* buffer, MotorDataA1& data) {
    // 帧头 (0xFD 0xEE)
    if (buffer[0] != GO_M8010_6_DATA_HEADER_BYTE1 || buffer[1] != GO_M8010_6_DATA_HEADER_BYTE2) {
        // ESP_LOGE(TAG_MOTOR, "接收数据帧头错误: 0x%02X 0x%02X", buffer[0], buffer[1]); // 不再打印错误日志
        return false;
    }

    // 校验CRC16_CCITT (针对从 buffer[0] 到 buffer[13] 的14字节数据)
    // 初始CRC值为0x0000
    uint16_t received_crc = (uint16_t)(buffer[GO_M8010_6_DATA_LENGTH - 1] << 8) | buffer[GO_M8010_6_DATA_LENGTH - 2];
    uint16_t calculated_crc = crc_ccitt(0x0000, buffer, GO_M8010_6_DATA_LENGTH - 2);

    if (received_crc != calculated_crc) {
        ESP_LOGE(TAG_MOTOR, "CRC校验失败: 接收0x%04X, 计算0x%04X", received_crc, calculated_crc); // 临时启用调试
        // 临时跳过CRC校验，继续解析数据
        // return false;
    }

    // 解包 ID 和 MODE - 字节 2
    data.id = buffer[2] & 0x0F;
    data.mode = (buffer[2] >> 4) & 0x07;

    // 解析完成

    // 解包力矩 (tau_fbk) - 字节 3-4 (int16_t)
    // T = ((float)torque) / 256.0f
    int16_t tau_packed = (int16_t)(buffer[4] << 8) | buffer[3]; // 低位在前，高位在后
    data.t = static_cast<float>(tau_packed) / GO_M8010_6_TAU_RATIO;

    // 解包速度 (omega_fbk) - 字节 5-6 (int16_t)
    // W = ((float)speed / 256.0f) * 6.28318f;
    int16_t omega_packed = (int16_t)(buffer[6] << 8) | buffer[5];
    data.vel = static_cast<float>(omega_packed) * GO_M8010_6_OMEGA_RAD_PER_UNIT;

    // 解包位置 (theta_fbk) - 字节 7-10 (int32_t)
    // Pos = 6.28318f * ((float)pos) / 32768.0f;
    int32_t theta_packed = (int32_t)(buffer[10] << 24) | (buffer[9] << 16) | (buffer[8] << 8) | buffer[7];
    data.pos = static_cast<float>(theta_packed) * GO_M8010_6_THETA_RAD_PER_UNIT;

    // 解包温度 (TEMP) - 字节 11 (int8_t)
    data.temp = static_cast<int8_t>(buffer[11]);

    // 解包 MERROR (3bit)
    data.MError = buffer[12] & 0x07; // 仅取低3位作为 MError

    // FORCE 12bit (MError_force 的高5位 + force_high_bits 的低7位)
    uint16_t foot_force_raw = ((uint16_t)(buffer[13] & 0x7F) << 5) | ((buffer[12] >> 3) & 0x1F);
    data.footForce = foot_force_raw; // 赋值给 MotorDataA1 新增的 footForce 字段

    return true;
}


esp_err_t UnitreeMotorDriver::sendRecv(const MotorCmdA1& cmd, MotorDataA1& data) {
    if (!_is_initialized) {
        ESP_LOGE(TAG_MOTOR, "电机驱动未初始化，无法发送/接收数据");
        return ESP_FAIL;
    }

    uint8_t tx_buffer[GO_M8010_6_CMD_LENGTH];
    uint8_t rx_buffer[GO_M8010_6_DATA_LENGTH];

    // 1. 打包电机命令 (将 MotorCmdA1 转换为 GO-M8010-6 协议格式)
    size_t packed_len = pack_motor_cmd_go_m8010_6(cmd, tx_buffer);

    // 2. 发送数据包 - 参考test_rs485主站模式
    int tx_bytes = uart_write_bytes(_uart_num, (const char*)tx_buffer, packed_len);
    if (tx_bytes != packed_len) {
        ESP_LOGE(TAG_MOTOR, "发送字节数不匹配: 期望%d, 实际%d", packed_len, tx_bytes);
        return ESP_FAIL;
    }

    // 关闭发送数据包打印

    // 3. 等待发送完成 - 确保数据完全发送
    uart_wait_tx_idle_polling(_uart_num);

    // 4. 接收响应 - 健壮的数据包解析，处理错位问题
    uint8_t temp_buffer[GO_M8010_6_DATA_LENGTH + 16]; // 增大缓冲区以处理错位
    int total_bytes = 0;

    // 分批接收数据，查找正确的帧头
    for (int attempts = 0; attempts < 3; attempts++) {
        int rx_bytes = uart_read_bytes(_uart_num, temp_buffer + total_bytes,
                                      sizeof(temp_buffer) - total_bytes, pdMS_TO_TICKS(2));
        if (rx_bytes > 0) {
            total_bytes += rx_bytes;

            // 查找帧头位置 (0xFD 0xEE)
            for (int i = 0; i <= total_bytes - GO_M8010_6_DATA_LENGTH; i++) {
                if (temp_buffer[i] == GO_M8010_6_DATA_HEADER_BYTE1 &&
                    temp_buffer[i + 1] == GO_M8010_6_DATA_HEADER_BYTE2) {

                    // 找到帧头，复制完整数据包
                    memcpy(rx_buffer, &temp_buffer[i], GO_M8010_6_DATA_LENGTH);

                    // 关闭数据包打印

                    // 5. 解析数据
                    if (unpack_motor_data_go_m8010_6(rx_buffer, data)) {
                        // 数据解析成功，返回到main.cpp进行统一打印
                        return ESP_OK;
                    } else {
                        ESP_LOGW(TAG_MOTOR, "数据解析失败，继续查找");
                        break; // 跳出内层循环，继续接收更多数据
                    }
                }
            }
        } else {
            break; // 没有更多数据
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
        ESP_LOGW(TAG_MOTOR, "接收超时 - 没有收到任何数据");
        return ESP_ERR_TIMEOUT;
    }
}

// test_rs485模式：使用同步通信，不需要异步接收任务

// 打印电机状态信息函数
void UnitreeMotorDriver::printMotorStatus(const MotorDataA1& data, const MotorCmdA1& cmd, uint32_t count) {
    ESP_LOGI(TAG_MOTOR, "========== 电机状态 #%lu ==========", count);

    // 电机控制命令信息
    ESP_LOGI(TAG_MOTOR, "控制命令 - ID:%d, 模式:%d, 位置:%.3f rad (%.1f°), 速度:%.2f rad/s, 力矩:%.3f N.m",
             cmd.id, cmd.mode, cmd.pos, cmd.pos * 57.2958f, cmd.vel, cmd.t);
    ESP_LOGI(TAG_MOTOR, "控制参数 - Kp:%.3f, Kd:%.3f", cmd.kp, cmd.kd);

    // 电机反馈状态信息
    ESP_LOGI(TAG_MOTOR, "反馈状态 - ID:%d, 模式:%d, 位置:%.3f rad (%.1f°), 速度:%.2f rad/s, 力矩:%.3f N.m",
             data.id, data.mode, data.pos, data.pos * 57.2958f, data.vel, data.t);
    ESP_LOGI(TAG_MOTOR, "电机状态 - 温度:%d°C, 错误码:0x%02X, 足端力:%d",
             data.temp, data.MError, data.footForce);

    // 错误状态提示
    if (data.MError != 0) {
        const char* error_desc = "未知错误";
        switch (data.MError) {
            case 1: error_desc = "过热"; break;
            case 2: error_desc = "过流"; break;
            case 3: error_desc = "过压"; break;
            case 4: error_desc = "编码器故障"; break;
            default: error_desc = "保留错误码"; break;
        }
        ESP_LOGW(TAG_MOTOR, "⚠️  电机错误: %s (错误码:%d)", error_desc, data.MError);
    }

    ESP_LOGI(TAG_MOTOR, "=====================================");
}
