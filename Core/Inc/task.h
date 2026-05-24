#ifndef __TASK_H
#define __TASK_H

#include "common.h"
#include "chassis.h"
#include "actuator.h"
#include "positioning.h"
#include "vision.h"
#include "navigation.h"

/* ========== Task State Machine ========== */
typedef enum {
    /* Phase 1: Pick from warehouse, deliver to numbered shelves */
    TASK_P1_SCAN_BARCODE,
    TASK_P1_GOTO_WAREHOUSE,
    TASK_P1_ALIGN_WAREHOUSE,
    TASK_P1_PICK_MATERIAL,
    TASK_P1_GOTO_SHELF,
    TASK_P1_PLACE_MATERIAL,

    /* Phase 2: Pick from material shelves, deliver to numbered shelves */
    TASK_P2_GOTO_PICK_SHELF,
    TASK_P2_ALIGN_PICK_SHELF,
    TASK_P2_PICK_MATERIAL,
    TASK_P2_GOTO_PLACE_SHELF,
    TASK_P2_PLACE_MATERIAL,

    /* Phase 3: Return to start zone */
    TASK_RETURN_HOME,
    TASK_DONE
} task_state_t;

/* ========== Arena Waypoints (meters) ========== */
/*
 * Coordinate system: set according to your arena layout
 * Origin at one corner, X = toward material area, Y = left
 *
 * TODO: Measure and set actual coordinates for your arena
 */
#define START_X       0.3f
#define START_Y       1.0f
#define START_ANGLE   0.0f

/* Warehouse positions (where materials are in the warehouse) */
#define WAREHOUSE_1_X  1.2f
#define WAREHOUSE_1_Y  0.3f
#define WAREHOUSE_2_X  1.2f
#define WAREHOUSE_2_Y  0.6f
#define WAREHOUSE_3_X  1.2f
#define WAREHOUSE_3_Y  0.9f
#define WAREHOUSE_4_X  1.2f
#define WAREHOUSE_4_Y  1.2f

/* Material shelf positions (numbered shelves 1-7) */
#define SHELF_1_X  2.0f
#define SHELF_1_Y  0.3f
#define SHELF_2_X  2.0f
#define SHELF_2_Y  0.6f
#define SHELF_3_X  2.0f
#define SHELF_3_Y  0.9f
#define SHELF_4_X  2.0f
#define SHELF_4_Y  1.2f
#define SHELF_5_X  2.5f
#define SHELF_5_Y  0.3f
#define SHELF_6_X  2.5f
#define SHELF_6_Y  0.6f
#define SHELF_7_X  2.5f
#define SHELF_7_Y  0.9f

/* ========== Material Tracking ========== */
#define MAX_MATERIALS  4
#define INVALID_SHELF  0

/* ========== Functions ========== */
void task_init(void);
void task_update(void);
task_state_t task_get_state(void);

/* ========== State ========== */
extern task_state_t task_state;

#endif /* __TASK_H */
