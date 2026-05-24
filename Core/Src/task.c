#include "task.h"

task_state_t task_state = TASK_P1_SCAN_BARCODE;

static uint8_t material_shelf[MAX_MATERIALS] = {0};
static uint8_t current_material = 0;
static uint8_t current_pick_shelf = 0;
static uint32_t state_enter_tick = 0;
static uint8_t  action_step = 0;
static uint32_t step_tick = 0;

/* ========== Position helpers ========== */
static void get_warehouse_pos(uint8_t idx, float *x, float *y)
{
    static const float wx[] = {WAREHOUSE_1_X, WAREHOUSE_2_X, WAREHOUSE_3_X, WAREHOUSE_4_X};
    static const float wy[] = {WAREHOUSE_1_Y, WAREHOUSE_2_Y, WAREHOUSE_3_Y, WAREHOUSE_4_Y};
    if (idx < MAX_MATERIALS) { *x = wx[idx]; *y = wy[idx]; }
    else { *x = START_X; *y = START_Y; }
}

static void get_shelf_pos(uint8_t shelf_num, float *x, float *y)
{
    static const float sx[] = {0, SHELF_1_X, SHELF_2_X, SHELF_3_X, SHELF_4_X, SHELF_5_X, SHELF_6_X, SHELF_7_X};
    static const float sy[] = {0, SHELF_1_Y, SHELF_2_Y, SHELF_3_Y, SHELF_4_Y, SHELF_5_Y, SHELF_6_Y, SHELF_7_Y};
    if (shelf_num >= 1 && shelf_num <= 7) { *x = sx[shelf_num]; *y = sy[shelf_num]; }
    else { *x = START_X; *y = START_Y; }
}

static void enter_state(task_state_t new_state)
{
    task_state = new_state;
    state_enter_tick = HAL_GetTick();
    action_step = 0;
    step_tick = HAL_GetTick();
}

static bool step_done(uint32_t ms) { return (HAL_GetTick() - step_tick) >= ms; }

/* ========== Pick: lower -> grip -> raise. Returns true when done ========== */
static bool do_pick(void)
{
    switch (action_step) {
    case 0: lift_move_down(); action_step = 1; step_tick = HAL_GetTick(); break;
    case 1: if (step_done(1000)) { gripper_close(); action_step = 2; step_tick = HAL_GetTick(); } break;
    case 2: if (step_done(500))  { lift_move_up();  action_step = 3; step_tick = HAL_GetTick(); } break;
    case 3: if (step_done(1000)) { lift_stop();     action_step = 4; } break;
    case 4: return true;
    }
    return false;
}

/* ========== Place: lower -> release -> raise. Returns true when done ========== */
static bool do_place(void)
{
    switch (action_step) {
    case 0: lift_move_down(); action_step = 1; step_tick = HAL_GetTick(); break;
    case 1: if (step_done(1000)) { gripper_open();  action_step = 2; step_tick = HAL_GetTick(); } break;
    case 2: if (step_done(500))  { lift_move_up();   action_step = 3; step_tick = HAL_GetTick(); } break;
    case 3: if (step_done(1000)) { lift_stop();      action_step = 4; } break;
    case 4: return true;
    }
    return false;
}

void task_init(void)
{
    current_material = 0;
    current_pick_shelf = 0;
    memset(material_shelf, 0, sizeof(material_shelf));
    enter_state(TASK_P1_SCAN_BARCODE);
}

void task_update(void)
{
    nav_update();
    positioning_process();
    vision_process();

    float tx, ty;

    switch (task_state) {

    /* ========== PHASE 1: warehouse -> numbered shelf ========== */

    case TASK_P1_SCAN_BARCODE:
        vision_request_barcode();
        enter_state(TASK_P1_GOTO_WAREHOUSE);
        break;

    case TASK_P1_GOTO_WAREHOUSE:
        if (vision_data_available()) {
            get_warehouse_pos(current_material, &tx, &ty);
            nav_goto(tx, ty);
            enter_state(TASK_P1_ALIGN_WAREHOUSE);
        }
        break;

    case TASK_P1_ALIGN_WAREHOUSE:
        if (nav_get_state() == NAV_REACHED) {
            vision_request_position();
            enter_state(TASK_P1_PICK_MATERIAL);
        }
        break;

    case TASK_P1_PICK_MATERIAL:
        if (vision_data_available() && vision_current.detected) {
            nav_goto(pos_current.x + vision_current.offset_x,
                     pos_current.y + vision_current.offset_y);
        }
        if (nav_get_state() == NAV_REACHED || (HAL_GetTick() - state_enter_tick > 3000)) {
            enter_state(TASK_P1_GOTO_SHELF);
        }
        break;

    case TASK_P1_GOTO_SHELF:
        if (action_step < 4 && do_pick()) {
            uint8_t sn = current_material + 1;
            material_shelf[current_material] = sn;
            get_shelf_pos(sn, &tx, &ty);
            nav_goto(tx, ty);
            enter_state(TASK_P1_PLACE_MATERIAL);
        }
        break;

    case TASK_P1_PLACE_MATERIAL:
        if (nav_get_state() == NAV_REACHED && action_step < 4) {
            if (do_place()) {
                current_material++;
                if (current_material < MAX_MATERIALS) {
                    enter_state(TASK_P1_SCAN_BARCODE);
                } else {
                    current_material = 0;
                    current_pick_shelf = 4;
                    enter_state(TASK_P2_GOTO_PICK_SHELF);
                }
            }
        }
        break;

    /* ========== PHASE 2: material shelf -> next shelf ========== */

    case TASK_P2_GOTO_PICK_SHELF:
        if (current_pick_shelf < 2) {
            enter_state(TASK_RETURN_HOME);
            break;
        }
        get_shelf_pos(current_pick_shelf, &tx, &ty);
        nav_goto(tx, ty);
        enter_state(TASK_P2_ALIGN_PICK_SHELF);
        break;

    case TASK_P2_ALIGN_PICK_SHELF:
        if (nav_get_state() == NAV_REACHED) {
            vision_request_position();
            enter_state(TASK_P2_PICK_MATERIAL);
        }
        break;

    case TASK_P2_PICK_MATERIAL:
        if (vision_data_available() && vision_current.detected) {
            nav_goto(pos_current.x + vision_current.offset_x,
                     pos_current.y + vision_current.offset_y);
        }
        if ((nav_get_state() == NAV_REACHED || (HAL_GetTick() - state_enter_tick > 3000))
            && action_step < 4) {
            if (do_pick()) {
                /* Pick done, navigate to place shelf */
                uint8_t ps = 5 + current_material;
                get_shelf_pos(ps, &tx, &ty);
                nav_goto(tx, ty);
                enter_state(TASK_P2_PLACE_MATERIAL);
            }
        }
        break;

    case TASK_P2_PLACE_MATERIAL:
        if (nav_get_state() == NAV_REACHED && action_step < 4) {
            if (do_place()) {
                current_material++;
                current_pick_shelf--;
                enter_state(TASK_P2_GOTO_PICK_SHELF);
            }
        }
        break;

    /* ========== PHASE 3: return home ========== */

    case TASK_RETURN_HOME:
        nav_goto_heading(START_X, START_Y, START_ANGLE);
        enter_state(TASK_DONE);
        break;

    case TASK_DONE:
        if (nav_get_state() == NAV_REACHED) {
            chassis_stop();
        }
        break;
    }
}

task_state_t task_get_state(void)
{
    return task_state;
}
