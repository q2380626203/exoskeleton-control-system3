#ifndef __CRC_CCITT_H
#define __CRC_CCITT_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

// CRC-CCITT 表，由 crc_ccitt.c 提供
extern const uint16_t crc_ccitt_table[256];

/**
 * @brief 计算单个字节的 CRC-CCITT 值
 * @param crc 当前的 CRC 值
 * @param c 要处理的字节
 * @return 更新后的 CRC 值
 */
uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);

/**
 * @brief 重新计算数据缓冲区的 CRC-CCITT (XMODEM) 校验值
 * @param crc 之前的 CRC 值 (通常初始化为 0x0000)
 * @param buffer 数据指针
 * @param len 缓冲区中的字节数
 * @return 计算出的 CRC-CCITT 校验值
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

#ifdef __cplusplus
}
#endif

#endif // __CRC_CCITT_H
