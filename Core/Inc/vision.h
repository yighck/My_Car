#ifndef __VISION_H
#define __VISION_H

#include "common.h"

/* ========== Vision Data Types ========== */
typedef enum {
    VISION_TYPE_BARCODE,    /* Barcode scan result */
    VISION_TYPE_POSITION,   /* Target position from vision */
    VISION_TYPE_RECOGNIZE   /* Material recognition */
} vision_type_t;

typedef struct {
    vision_type_t type;
    uint16_t      barcode_id;   /* Decoded barcode number */
    float         offset_x;     /* X offset from camera center (m) */
    float         offset_y;     /* Y offset from camera center (m) */
    bool          detected;     /* Target detected flag */
    uint32_t      timestamp;
} vision_data_t;

/* ========== Protocol ========== */
/*
 * Linux vision sends structured data via USART1:
 * [0xCC][TYPE][DATA...][0x33]
 *
 * Barcode response:    [0xCC][0x01][ID_H][ID_L][CHECK][0x33]
 * Position response:   [0xCC][0x02][X(4)][Y(4)][FOUND(1)][CHECK][0x33]
 * Recognize response:  [0xCC][0x03][ID_H][ID_L][X(4)][Y(4)][CHECK][0x33]
 *
 * Send to Linux:       [0xDD][CMD][PARAM...][CHECK][0x33]
 *   CMD 0x01: scan barcode
 *   CMD 0x02: find target position
 *   CMD 0x03: recognize material
 *
 * TODO: Verify with your Linux vision program
 */
#define VISION_RX_HEADER  0xCC
#define VISION_TX_HEADER  0xDD
#define VISION_TAIL       0x33

/* ========== Functions ========== */
void vision_init(void);
void vision_process(void);
void vision_request_barcode(void);
void vision_request_position(void);
void vision_request_recognize(void);
bool vision_data_available(void);

/* ========== Data ========== */
extern vision_data_t vision_current;

#endif /* __VISION_H */
