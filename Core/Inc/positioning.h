#ifndef __POSITIONING_H
#define __POSITIONING_H

#include "common.h"

/* ========== OPS9 Position Data ========== */
typedef struct {
    float x;        /* Global X (m)     */
    float y;        /* Global Y (m)     */
    float angle;    /* Heading (rad)    */
    bool  valid;    /* Data valid flag  */
    uint32_t timestamp;
} pos_data_t;

/* ========== OPS9 Protocol (from manual) ========== */
/*
 * 20 bytes, little-endian:
 * [0xAA][0x00][X(4B float,mm)][Y(4B float,mm)][Z(4B float,deg)][Time(4B uint32,us)][Checksum(2B)]
 * Checksum = SUM of bytes[2..17], low 16 bits
 * Default output: 100Hz
 * Baud rate: 921600
 */
#define OPS9_HEADER1        0xAA
#define OPS9_HEADER2        0x00
#define OPS9_FRAME_LEN      20
#define OPS9_BAUD_RATE      921600

/* ========== Functions ========== */
void positioning_init(void);
void positioning_process(void);
bool positioning_is_valid(void);

/* ========== Data ========== */
extern pos_data_t pos_current;

#endif /* __POSITIONING_H */
