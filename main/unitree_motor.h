#ifndef UNITREE_MOTOR_H
#define UNITREE_MOTOR_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "motor_commands.h"

class UnitreeMotorDriver {
public:
    UnitreeMotorDriver();
    ~UnitreeMotorDriver();

    /**
     * @brief 初始化UART串口和电机驱动
     * @param uart_num UART端口号 (e.g., UART_NUM_2)
     * @param tx_pin TX引脚号
     * @param rx_pin RX引脚号
     * @param max485_re_de_pin 未使用，传入GPIO_NUM_NC（外部RS485模块自动控制）
     * @param baud_rate 波特率
     * @return true 成功, false 失败
     */
    bool init(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t max485_re_de_pin, int baud_rate);

    /**
     * @brief 发送电机控制命令并接收反馈 (兼容旧的A1指令格式，内部转换为GO-M8010-6)
     * @param cmd 要发送的电机命令 (MotorCmdA1 格式)
     * @param data 接收到的电机反馈数据 (MotorDataA1 格式)
     * @return ESP_OK 成功, 其他错误码 失败
     */
    esp_err_t sendRecv(const MotorCmdA1& cmd, MotorDataA1& data);

    /**
     * @brief 检查电机驱动是否已初始化
     * @return true 已初始化, false 未初始化
     */
    bool isInitialized() const { return _is_initialized; }

private:
    uart_port_t _uart_num;
    gpio_num_t _max485_re_de_pin;
    bool _is_initialized;

    /**
     * @brief 将 MotorCmdA1 结构体转换为 MotorCmdGO_M8010_6 并打包成字节数组
     * @param cmd 要打包的命令 (MotorCmdA1 格式)
     * @param buffer 输出字节缓冲区
     * @return 打包后的数据长度
     */
    size_t pack_motor_cmd_go_m8010_6(const MotorCmdA1& cmd, uint8_t* buffer);

    /**
     * @brief 将字节数组解包成 MotorDataGO_M8010_6 并转换为 MotorDataA1 结构体
     * @param buffer 输入字节缓冲区
     * @param data 输出反馈数据 (MotorDataA1 格式)
     * @return true 成功, false 失败 (例如CRC校验失败)
     */
    bool unpack_motor_data_go_m8010_6(const uint8_t* buffer, MotorDataA1& data);
};

#endif // UNITREE_MOTOR_H
