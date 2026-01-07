#include "tcp_replay_receive.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

/* ==================== 协议结构定义 ==================== */

/* 回放数据包头（14字节，与上传协议相同） */
#pragma pack(push, 1)
typedef struct {
    uint16_t magic;             /* 0xBB66 */
    uint8_t device_id;
    uint8_t version;            /* 10 */
    uint8_t seq;
    uint8_t flags;              /* 高4位=命令, 低4位=标志 */
    uint8_t count;              /* 数据条数 (0-100) */
    uint8_t reserved;
    uint8_t base_time[6];       /* 48bit毫秒时间戳 */
} replay_packet_header_t;
#pragma pack(pop)

/* 单条数据格式 - 不含roll (15字节) */
#pragma pack(push, 1)
typedef struct {
    uint16_t time_offset_ms;
    int16_t pos1;
    int16_t pos2;
    int16_t vel1;
    int16_t vel2;
    int16_t torque1;
    int16_t torque2;
    uint8_t states;
} replay_sample_raw_t;
#pragma pack(pop)

/* 单条数据格式 - 含roll (19字节) */
#pragma pack(push, 1)
typedef struct {
    uint16_t time_offset_ms;
    int16_t pos1;
    int16_t pos2;
    int16_t vel1;
    int16_t vel2;
    int16_t torque1;
    int16_t torque2;
    int16_t roll_left;
    int16_t roll_right;
    uint8_t states;
} replay_sample_with_roll_raw_t;
#pragma pack(pop)

#define SAMPLE_SIZE_NO_ROLL     15
#define SAMPLE_SIZE_WITH_ROLL   19
#define HEADER_SIZE             14
#define BT_IMU_NOT_CONNECTED    0x7FFF

/* ==================== 环形缓冲区 ==================== */

static replay_sample_t s_buffer[REPLAY_BUFFER_SIZE];
static volatile int s_write_index = 0;
static volatile int s_read_index = 0;
static volatile int s_count = 0;
static SemaphoreHandle_t s_buffer_mutex = NULL;

/* ==================== 回放状态 ==================== */

static volatile replay_state_t s_state = REPLAY_STATE_IDLE;
static volatile bool s_is_last_packet_received = false;
static volatile uint64_t s_base_timestamp_ms = 0;       /* 第一包的基准时间戳 */
static volatile uint64_t s_replay_start_time_ms = 0;    /* 回放开始时的本地时间 */

/* ==================== 统计信息 ==================== */

static volatile uint32_t s_total_received = 0;
static volatile uint32_t s_total_played = 0;
static volatile uint32_t s_packets_received = 0;

/* ==================== 参数回调和响应缓存 ==================== */

static tcp_replay_param_set_cb_t s_param_set_cb = NULL;
static tcp_replay_param_get_cb_t s_param_get_cb = NULL;

/* 待发送的参数响应缓存 */
#define PARAM_RESPONSE_BUFFER_SIZE 128
static uint8_t s_pending_response[PARAM_RESPONSE_BUFFER_SIZE];
static volatile int s_pending_response_len = 0;

/* 所有支持的参数ID列表 */
static const uint8_t s_all_param_ids[] = {
    PARAM_M1_TRIGGER_SPEED, PARAM_M1_PHASE1_TORQUE, PARAM_M1_PHASE1_KD,
    PARAM_M1_PHASE2_TORQUE, PARAM_M1_PHASE2_KD, PARAM_M1_PASSIVE_KD,
    PARAM_M2_TRIGGER_SPEED, PARAM_M2_PHASE1_TORQUE, PARAM_M2_PHASE1_KD,
    PARAM_M2_PHASE2_TORQUE, PARAM_M2_PHASE2_KD, PARAM_M2_PASSIVE_KD,
    PARAM_VELOCITY_SCALE, PARAM_VELOCITY_LIMIT, PARAM_PHASE1_TIMEOUT,
    PARAM_PHASE2_TIMEOUT
};
#define ALL_PARAM_COUNT (sizeof(s_all_param_ids) / sizeof(s_all_param_ids[0]))

/* ==================== CRC8校验 ==================== */

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

static uint8_t calc_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

/* ==================== 内部函数 ==================== */

/**
 * @brief 向缓冲区添加一条数据
 */
static bool buffer_push(const replay_sample_t *sample)
{
    if (s_count >= REPLAY_BUFFER_SIZE) {
        return false;  /* 缓冲区满 */
    }

    memcpy(&s_buffer[s_write_index], sample, sizeof(replay_sample_t));
    s_write_index = (s_write_index + 1) % REPLAY_BUFFER_SIZE;
    s_count++;

    return true;
}

/**
 * @brief 从缓冲区取出一条数据
 */
static bool buffer_pop(replay_sample_t *sample)
{
    if (s_count <= 0) {
        return false;  /* 缓冲区空 */
    }

    memcpy(sample, &s_buffer[s_read_index], sizeof(replay_sample_t));
    s_read_index = (s_read_index + 1) % REPLAY_BUFFER_SIZE;
    s_count--;

    return true;
}

/**
 * @brief 清空缓冲区
 */
static void buffer_clear(void)
{
    s_write_index = 0;
    s_read_index = 0;
    s_count = 0;
}

/**
 * @brief 解析单条数据（不含roll）
 */
static void parse_sample_no_roll(const replay_sample_raw_t *raw, uint64_t base_time, replay_sample_t *out)
{
    out->timestamp_offset_ms = raw->time_offset_ms;
    out->motor1_pos = raw->pos1 / 100.0f;
    out->motor2_pos = raw->pos2 / 100.0f;
    out->motor1_vel = raw->vel1 / 100.0f;
    out->motor2_vel = raw->vel2 / 100.0f;
    out->motor1_torque = raw->torque1 / 100.0f;
    out->motor2_torque = raw->torque2 / 100.0f;
    out->roll_left = NAN;
    out->roll_right = NAN;
    out->m1_state_label = (raw->states >> 4) & 0x0F;
    out->m2_state_label = raw->states & 0x0F;
}

/**
 * @brief 解析单条数据（含roll）
 */
static void parse_sample_with_roll(const replay_sample_with_roll_raw_t *raw, uint64_t base_time, replay_sample_t *out)
{
    out->timestamp_offset_ms = raw->time_offset_ms;
    out->motor1_pos = raw->pos1 / 100.0f;
    out->motor2_pos = raw->pos2 / 100.0f;
    out->motor1_vel = raw->vel1 / 100.0f;
    out->motor2_vel = raw->vel2 / 100.0f;
    out->motor1_torque = raw->torque1 / 100.0f;
    out->motor2_torque = raw->torque2 / 100.0f;
    out->roll_left = (raw->roll_left == BT_IMU_NOT_CONNECTED) ? NAN : (raw->roll_left / 100.0f);
    out->roll_right = (raw->roll_right == BT_IMU_NOT_CONNECTED) ? NAN : (raw->roll_right / 100.0f);
    out->m1_state_label = (raw->states >> 4) & 0x0F;
    out->m2_state_label = raw->states & 0x0F;
}

/**
 * @brief 处理命令
 */
static void handle_command(uint8_t cmd)
{
    switch (cmd) {
        case REPLAY_CMD_START:
            /* 开始回放 */
            buffer_clear();
            s_total_received = 0;
            s_total_played = 0;
            s_packets_received = 0;
            s_is_last_packet_received = false;
            s_base_timestamp_ms = 0;
            s_replay_start_time_ms = 0;
            s_state = REPLAY_STATE_BUFFERING;
            break;

        case REPLAY_CMD_STOP:
            /* 停止回放 */
            s_state = REPLAY_STATE_IDLE;
            buffer_clear();
            break;

        case REPLAY_CMD_PAUSE:
            /* 暂停回放 */
            if (s_state == REPLAY_STATE_PLAYING) {
                s_state = REPLAY_STATE_PAUSED;
            } else if (s_state == REPLAY_STATE_PAUSED) {
                s_state = REPLAY_STATE_PLAYING;
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 处理参数设置命令
 * @param data 数据指针（从包头开始）
 * @param len 数据长度
 */
static void handle_set_param(const uint8_t *data, size_t len)
{
    const replay_packet_header_t *header = (const replay_packet_header_t *)data;
    uint8_t count = header->count;

    if (count == 0 || s_param_set_cb == NULL) {
        return;
    }

    /* 计算预期包大小: 14字节包头 + count*5字节参数数据 + 1字节CRC */
    size_t expected_size = HEADER_SIZE + count * 5 + 1;
    if (len < expected_size) {
        return;
    }

    /* 解析并设置参数 */
    const uint8_t *param_data = data + HEADER_SIZE;
    for (int i = 0; i < count; i++) {
        uint8_t param_id = param_data[i * 5];
        float value;
        memcpy(&value, &param_data[i * 5 + 1], sizeof(float));
        s_param_set_cb(param_id, value);
    }
}

/**
 * @brief 处理参数查询命令并构建响应
 * @param data 数据指针（从包头开始）
 * @param len 数据长度
 */
static void handle_query_param(const uint8_t *data, size_t len)
{
    if (s_param_get_cb == NULL) {
        return;
    }

    const replay_packet_header_t *header = (const replay_packet_header_t *)data;
    uint8_t count = header->count;

    const uint8_t *query_ids;
    uint8_t query_count;

    if (count == 0) {
        /* 查询所有参数 */
        query_ids = s_all_param_ids;
        query_count = ALL_PARAM_COUNT;
    } else {
        /* 查询指定参数 */
        query_ids = data + HEADER_SIZE;
        query_count = count;
    }

    /* 构建响应包 */
    s_pending_response_len = tcp_replay_build_param_response(
        s_pending_response, PARAM_RESPONSE_BUFFER_SIZE,
        query_ids, query_count);
}

/* ==================== 公共接口实现 ==================== */

esp_err_t tcp_replay_init(void)
{
    if (s_buffer_mutex == NULL) {
        s_buffer_mutex = xSemaphoreCreateMutex();
        if (s_buffer_mutex == NULL) {
            return ESP_FAIL;
        }
    }

    buffer_clear();
    s_state = REPLAY_STATE_IDLE;
    s_total_received = 0;
    s_total_played = 0;
    s_packets_received = 0;
    s_is_last_packet_received = false;

    return ESP_OK;
}

int tcp_replay_process_data(const uint8_t *data, size_t len)
{
    if (data == NULL || len < HEADER_SIZE + 1) {
        return -1;
    }

    /* 检查Magic */
    uint16_t magic = data[0] | (data[1] << 8);
    if (magic != REPLAY_MAGIC) {
        return 0;  /* 不是回放数据包，返回0表示未处理 */
    }

    /* 解析包头 */
    const replay_packet_header_t *header = (const replay_packet_header_t *)data;
    uint8_t cmd = header->flags & 0xF0;
    bool has_roll = (header->flags & REPLAY_FLAG_HAS_ROLL) != 0;
    bool is_last = (header->flags & REPLAY_FLAG_LAST) != 0;
    uint8_t count = header->count;

    /* 根据命令类型计算包大小 */
    size_t expected_size;
    if (cmd == REPLAY_CMD_SET_PARAM) {
        /* 参数设置包: 14字节包头 + count*5字节参数数据 + 1字节CRC */
        expected_size = HEADER_SIZE + count * 5 + 1;
    } else if (cmd == REPLAY_CMD_QUERY_PARAM) {
        /* 参数查询包: 14字节包头 + count*1字节参数ID + 1字节CRC */
        expected_size = HEADER_SIZE + count * 1 + 1;
    } else {
        /* 数据包或其他命令包 */
        size_t sample_size = has_roll ? SAMPLE_SIZE_WITH_ROLL : SAMPLE_SIZE_NO_ROLL;
        expected_size = HEADER_SIZE + count * sample_size + 1;  /* +1 for CRC */
    }

    if (len < expected_size) {
        return -1;  /* 数据不完整 */
    }

    /* 验证CRC */
    uint8_t crc_calc = calc_crc8(data, expected_size - 1);
    uint8_t crc_recv = data[expected_size - 1];
    if (crc_calc != crc_recv) {
        return -1;  /* CRC错误 */
    }

    /* 处理命令 */
    if (cmd != REPLAY_CMD_DATA) {
        if (cmd == REPLAY_CMD_SET_PARAM) {
            handle_set_param(data, len);
        } else if (cmd == REPLAY_CMD_QUERY_PARAM) {
            handle_query_param(data, len);
        } else {
            handle_command(cmd);
        }
        return expected_size;
    }

    /* 处理数据包 */
    if (s_state == REPLAY_STATE_IDLE) {
        return expected_size;  /* 未开始回放，丢弃数据 */
    }

    /* 解析基准时间戳 */
    uint64_t base_time = 0;
    for (int i = 0; i < 6; i++) {
        base_time |= ((uint64_t)header->base_time[i]) << (i * 8);
    }

    /* 记录第一包的基准时间戳 */
    if (s_base_timestamp_ms == 0 && count > 0) {
        s_base_timestamp_ms = base_time;
    }

    /* 解析并存储数据 */
    if (xSemaphoreTake(s_buffer_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        const uint8_t *sample_data = data + HEADER_SIZE;

        for (int i = 0; i < count; i++) {
            replay_sample_t sample;

            if (has_roll) {
                const replay_sample_with_roll_raw_t *raw =
                    (const replay_sample_with_roll_raw_t *)(sample_data + i * SAMPLE_SIZE_WITH_ROLL);
                parse_sample_with_roll(raw, base_time, &sample);
            } else {
                const replay_sample_raw_t *raw =
                    (const replay_sample_raw_t *)(sample_data + i * SAMPLE_SIZE_NO_ROLL);
                parse_sample_no_roll(raw, base_time, &sample);
            }

            /* 计算相对于回放开始的绝对时间偏移 */
            sample.timestamp_offset_ms = (uint32_t)(base_time - s_base_timestamp_ms) + sample.timestamp_offset_ms;

            if (buffer_push(&sample)) {
                s_total_received++;
            }
        }

        s_packets_received++;

        /* 检查是否收到足够数据可以开始回放 */
        if (s_state == REPLAY_STATE_BUFFERING && s_count >= REPLAY_LOW_WATERMARK) {
            s_state = REPLAY_STATE_PLAYING;
            s_replay_start_time_ms = esp_timer_get_time() / 1000;
        }

        /* 检查是否是最后一包 */
        if (is_last) {
            s_is_last_packet_received = true;
        }

        xSemaphoreGive(s_buffer_mutex);
    }

    return expected_size;
}

bool tcp_replay_get_next_sample(replay_sample_t *sample)
{
    if (sample == NULL) {
        return false;
    }

    if (s_state != REPLAY_STATE_PLAYING) {
        return false;
    }

    if (s_buffer_mutex == NULL) {
        return false;
    }

    bool got_sample = false;

    if (xSemaphoreTake(s_buffer_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        /* 直接取出下一条数据，时序控制由调用者负责 */
        if (s_count > 0) {
            buffer_pop(sample);
            s_total_played++;
            got_sample = true;
        }

        /* 检查是否回放完成 */
        if (s_count == 0 && s_is_last_packet_received) {
            s_state = REPLAY_STATE_FINISHED;
        }

        xSemaphoreGive(s_buffer_mutex);
    }

    return got_sample;
}

uint16_t tcp_replay_get_next_interval_ms(void)
{
    if (s_state != REPLAY_STATE_PLAYING || s_buffer_mutex == NULL) {
        return 0;
    }

    uint16_t interval = 0;

    if (xSemaphoreTake(s_buffer_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (s_count >= 2) {
            /* 获取当前数据和下一条数据的时间戳 */
            int next_index = (s_read_index + 1) % REPLAY_BUFFER_SIZE;
            uint32_t current_ts = s_buffer[s_read_index].timestamp_offset_ms;
            uint32_t next_ts = s_buffer[next_index].timestamp_offset_ms;

            /* 计算间隔 */
            if (next_ts > current_ts) {
                interval = (uint16_t)(next_ts - current_ts);
            }
            /* 如果间隔为0或异常，使用默认间隔 */
            if (interval == 0 || interval > 100) {
                interval = 7;  /* 默认7ms间隔 */
            }
        } else if (s_count == 1) {
            /* 只有一条数据，使用默认间隔 */
            interval = 7;
        }
        xSemaphoreGive(s_buffer_mutex);
    }

    return interval;
}

replay_state_t tcp_replay_get_state(void)
{
    return s_state;
}

bool tcp_replay_is_active(void)
{
    return (s_state == REPLAY_STATE_BUFFERING ||
            s_state == REPLAY_STATE_PLAYING ||
            s_state == REPLAY_STATE_PAUSED);
}

int tcp_replay_get_buffer_count(void)
{
    return s_count;
}

void tcp_replay_get_stats(uint32_t *total_received, uint32_t *total_played)
{
    if (total_received) *total_received = s_total_received;
    if (total_played) *total_played = s_total_played;
}

void tcp_replay_reset(void)
{
    if (s_buffer_mutex && xSemaphoreTake(s_buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        buffer_clear();
        s_state = REPLAY_STATE_IDLE;
        s_total_received = 0;
        s_total_played = 0;
        s_packets_received = 0;
        s_is_last_packet_received = false;
        s_base_timestamp_ms = 0;
        s_replay_start_time_ms = 0;
        xSemaphoreGive(s_buffer_mutex);
    }
}

bool tcp_replay_need_more_data(void)
{
    /* 只有在回放活跃状态下才请求数据 */
    if (s_state != REPLAY_STATE_BUFFERING &&
        s_state != REPLAY_STATE_PLAYING &&
        s_state != REPLAY_STATE_PAUSED) {
        return false;
    }

    /* 已收到最后一包，不再请求 */
    if (s_is_last_packet_received) {
        return false;
    }

    /* 缓冲区数据量低于低水位时请求更多数据 */
    return (s_count < REPLAY_LOW_WATERMARK);
}

int tcp_replay_build_request_packet(uint8_t *buffer, size_t buffer_size)
{
    /* 请求包格式（15字节，与数据包头相同结构）：
     * - magic: 2 (0xBB66)
     * - device_id: 1
     * - version: 1 (10)
     * - seq: 1
     * - flags: 1 (高4位=0x40请求命令, 低4位=0)
     * - count: 1 (请求的包数，0=请求默认数量)
     * - reserved: 1
     * - base_time[6]: 6 (当前缓冲区数据量，供服务端参考)
     * - crc8: 1
     */
    if (buffer == NULL || buffer_size < 15) {
        return 0;
    }

    static uint8_t s_request_seq = 0;

    /* 构建包头 */
    buffer[0] = REPLAY_MAGIC & 0xFF;
    buffer[1] = (REPLAY_MAGIC >> 8) & 0xFF;
    buffer[2] = 0;  /* device_id, 由tcp_data_upload填充 */
    buffer[3] = 10; /* version */
    buffer[4] = s_request_seq++;
    buffer[5] = REPLAY_CMD_REQUEST;  /* flags: 请求命令 */
    buffer[6] = 1;  /* count: 请求1个包（100条数据） */
    buffer[7] = 0;  /* reserved */

    /* base_time字段用于传递当前缓冲区数据量 */
    uint16_t buffer_count = (uint16_t)s_count;
    buffer[8] = buffer_count & 0xFF;
    buffer[9] = (buffer_count >> 8) & 0xFF;
    buffer[10] = 0;
    buffer[11] = 0;
    buffer[12] = 0;
    buffer[13] = 0;

    /* 计算CRC8 */
    buffer[14] = calc_crc8(buffer, 14);

    return 15;
}

void tcp_replay_set_param_set_callback(tcp_replay_param_set_cb_t cb)
{
    s_param_set_cb = cb;
}

void tcp_replay_set_param_get_callback(tcp_replay_param_get_cb_t cb)
{
    s_param_get_cb = cb;
}

int tcp_replay_build_param_response(uint8_t *buffer, size_t buffer_size,
                                     const uint8_t *param_ids, uint8_t count)
{
    if (buffer == NULL || s_param_get_cb == NULL || count == 0) {
        return 0;
    }

    /* 响应包格式：14字节包头 + count*5字节参数数据 + 1字节CRC */
    size_t required_size = HEADER_SIZE + count * 5 + 1;
    if (buffer_size < required_size) {
        return 0;
    }

    static uint8_t s_response_seq = 0;

    /* 构建包头 */
    buffer[0] = REPLAY_MAGIC & 0xFF;
    buffer[1] = (REPLAY_MAGIC >> 8) & 0xFF;
    buffer[2] = 0;  /* device_id */
    buffer[3] = 10; /* version */
    buffer[4] = s_response_seq++;
    buffer[5] = REPLAY_CMD_PARAM_RESPONSE;  /* flags: 参数响应 */
    buffer[6] = count;
    buffer[7] = 0;  /* reserved */

    /* base_timestamp (6字节, 当前时间) */
    uint64_t current_time_ms = esp_timer_get_time() / 1000;
    buffer[8] = (current_time_ms >> 0) & 0xFF;
    buffer[9] = (current_time_ms >> 8) & 0xFF;
    buffer[10] = (current_time_ms >> 16) & 0xFF;
    buffer[11] = (current_time_ms >> 24) & 0xFF;
    buffer[12] = (current_time_ms >> 32) & 0xFF;
    buffer[13] = (current_time_ms >> 40) & 0xFF;

    /* 填充参数数据 */
    uint8_t *param_data = buffer + HEADER_SIZE;
    for (int i = 0; i < count; i++) {
        uint8_t param_id = param_ids[i];
        float value = s_param_get_cb(param_id);

        param_data[i * 5] = param_id;
        memcpy(&param_data[i * 5 + 1], &value, sizeof(float));
    }

    /* 计算CRC8 */
    buffer[required_size - 1] = calc_crc8(buffer, required_size - 1);

    return (int)required_size;
}

bool tcp_replay_has_pending_response(void)
{
    return s_pending_response_len > 0;
}

int tcp_replay_get_pending_response(uint8_t *buffer, size_t buffer_size)
{
    if (buffer == NULL || s_pending_response_len == 0) {
        return 0;
    }

    if (buffer_size < (size_t)s_pending_response_len) {
        return 0;
    }

    int len = s_pending_response_len;
    memcpy(buffer, s_pending_response, len);
    s_pending_response_len = 0;  /* 清除待发送标志 */

    return len;
}
