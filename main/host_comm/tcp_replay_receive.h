#ifndef TCP_REPLAY_RECEIVE_H
#define TCP_REPLAY_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ==================== 回放协议定义 ==================== */

/* 回放数据包Magic（区分于上传的0xAA55） */
#define REPLAY_MAGIC            0xBB66

/* 命令类型（flags高4位） */
#define REPLAY_CMD_DATA         0x00    /* 数据包 */
#define REPLAY_CMD_START        0x10    /* 开始回放 */
#define REPLAY_CMD_STOP         0x20    /* 停止回放 */
#define REPLAY_CMD_PAUSE        0x30    /* 暂停回放 */
#define REPLAY_CMD_REQUEST      0x40    /* 请求更多数据 (ESP32->Server) */

/* 标志位（flags低4位） */
#define REPLAY_FLAG_LAST        0x01    /* 最后一包 */
#define REPLAY_FLAG_HAS_ROLL    0x02    /* 包含roll数据 */

/* 回放缓冲区配置 */
#define REPLAY_BUFFER_SIZE      500     /* 环形缓冲区大小（条） */
#define REPLAY_LOW_WATERMARK    100     /* 低水位，触发请求更多数据 */

/* 单条回放数据（与上传协议对应） */
typedef struct {
    uint32_t timestamp_offset_ms;   /* 相对于回放开始的时间偏移(ms) */
    float motor1_pos;
    float motor1_vel;
    float motor1_torque;
    float motor2_pos;
    float motor2_vel;
    float motor2_torque;
    float roll_left;                /* NAN表示无效 */
    float roll_right;               /* NAN表示无效 */
    int8_t m1_state_label;
    int8_t m2_state_label;
} replay_sample_t;

/* 回放状态 */
typedef enum {
    REPLAY_STATE_IDLE,              /* 空闲，未开始 */
    REPLAY_STATE_BUFFERING,         /* 缓冲中，等待足够数据 */
    REPLAY_STATE_PLAYING,           /* 回放中 */
    REPLAY_STATE_PAUSED,            /* 暂停 */
    REPLAY_STATE_FINISHED           /* 回放完成 */
} replay_state_t;

/**
 * @brief 初始化回放接收模块
 * @return ESP_OK成功
 */
esp_err_t tcp_replay_init(void);

/**
 * @brief 处理接收到的TCP数据（在TCP接收回调中调用）
 * @param data 数据指针
 * @param len 数据长度
 * @return 已处理的字节数，-1表示错误
 */
int tcp_replay_process_data(const uint8_t *data, size_t len);

/**
 * @brief 获取下一条回放数据（强制获取，不检查时间）
 * @param sample 输出参数，回放数据
 * @return true有数据，false无数据
 */
bool tcp_replay_get_next_sample(replay_sample_t *sample);

/**
 * @brief 获取当前数据与下一条数据之间的时间间隔（用于精确时序控制）
 * @return 时间间隔(ms)，如果缓冲区为空或只有一条数据返回0
 */
uint16_t tcp_replay_get_next_interval_ms(void);

/**
 * @brief 获取当前回放状态
 */
replay_state_t tcp_replay_get_state(void);

/**
 * @brief 检查是否处于回放模式
 */
bool tcp_replay_is_active(void);

/**
 * @brief 获取缓冲区数据量
 */
int tcp_replay_get_buffer_count(void);

/**
 * @brief 获取回放统计信息
 * @param total_received 总接收条数
 * @param total_played 已回放条数
 */
void tcp_replay_get_stats(uint32_t *total_received, uint32_t *total_played);

/**
 * @brief 重置回放模块
 */
void tcp_replay_reset(void);

/**
 * @brief 检查是否需要请求更多数据
 * @return true需要请求，false不需要
 * @note 当缓冲区数据量低于LOW_WATERMARK且未收到最后一包时返回true
 */
bool tcp_replay_need_more_data(void);

/**
 * @brief 构建数据请求包
 * @param buffer 输出缓冲区，至少15字节
 * @param buffer_size 缓冲区大小
 * @return 请求包大小，失败返回0
 */
int tcp_replay_build_request_packet(uint8_t *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* TCP_REPLAY_RECEIVE_H */
