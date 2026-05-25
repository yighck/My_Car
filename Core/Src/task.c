#include "task.h"
#include <stdio.h>

#define NAV_ALIGN_TIMEOUT_MS   3000U
#define VISION_WAIT_TIMEOUT_MS 10000U
#define VISION_MAX_RETRIES     5U      /* 视觉识别最大重试次数 */

task_state_t task_state = TASK_INIT;

static uint32_t state_enter_tick = 0;
static uint8_t  action_step = 0;
static uint32_t step_tick = 0;

static uint8_t  motion_step = 0;
static uint32_t motion_tick = 0;

static uint8_t  vision_retry_cnt = 0;  /* 视觉重试计数 */
static bool     b1_nav_started = false; /* B1最后取料并行导航标志 */
static bool     b2_nav_started = false; /* B2最后取料并行导航标志 */

/* 抓取统计 */
task_stats_t task_stats = {0};

static void get_rough_pos(uint8_t pos_num, float *x, float *y)
{
    static const float rx[] = {0, ROUGH_1_X, ROUGH_2_X, ROUGH_3_X};
    static const float ry[] = {0, ROUGH_1_Y, ROUGH_2_Y, ROUGH_3_Y};

    if (pos_num >= 1 && pos_num <= 3) {
        *x = rx[pos_num];
        *y = ry[pos_num];
    } else {
        *x = START_X;
        *y = START_Y;
    }
}

static void get_temp_pos(uint8_t pos_num, float *x, float *y)
{
    static const float tx[] = {0, TEMP_1_X, TEMP_2_X, TEMP_3_X};
    static const float ty[] = {0, TEMP_1_Y, TEMP_2_Y, TEMP_3_Y};

    if (pos_num >= 1 && pos_num <= 3) {
        *x = tx[pos_num];
        *y = ty[pos_num];
    } else {
        *x = START_X;
        *y = START_Y;
    }
}

static uint8_t state_index3(task_state_t state,
                            task_state_t s0,
                            task_state_t s1,
                            task_state_t s2)
{
    if (state == s0) return 0;
    if (state == s1) return 1;
    if (state == s2) return 2;
    return 0;
}

static void reset_motion(void)
{
    motion_step = 0;
    motion_tick = HAL_GetTick();
}

static void set_action_step(uint8_t new_step)
{
    action_step = new_step;
    step_tick = HAL_GetTick();
    reset_motion();
}

static void enter_state(task_state_t new_state)
{
    task_state = new_state;
    state_enter_tick = HAL_GetTick();
    action_step = 0;
    step_tick = state_enter_tick;
    reset_motion();
}

static bool step_done(uint32_t ms)
{
    return (HAL_GetTick() - step_tick) >= ms;
}

static bool motion_done(uint32_t ms)
{
    return (HAL_GetTick() - motion_tick) >= ms;
}

static void set_motion_step(uint8_t new_step)
{
    motion_step = new_step;
    motion_tick = HAL_GetTick();
}

static void prepare_tray_slot(uint8_t slot)
{
    gripper_rotate_to_tray();
    tray_rotate_to_slot(slot);
}

static bool do_pick(void)
{
    switch (motion_step) {
    case 0:
        lift_move_down();
        set_motion_step(1);
        break;
    case 1:
        if (motion_done(1000)) {
            gripper_close();
            set_motion_step(2);
        }
        break;
    case 2:
        if (motion_done(500)) {
            lift_move_up();
            set_motion_step(3);
        }
        break;
    case 3:
        if (motion_done(1000)) {
            lift_stop();
            set_motion_step(4);
        }
        break;
    case 4:
        return true;
    default:
        reset_motion();
        break;
    }

    return false;
}

static bool do_place_with_down_time(uint32_t down_ms)
{
    switch (motion_step) {
    case 0:
        lift_move_down();
        set_motion_step(1);
        break;
    case 1:
        if (motion_done(down_ms)) {
            gripper_open();
            set_motion_step(2);
        }
        break;
    case 2:
        if (motion_done(500)) {
            lift_move_up();
            set_motion_step(3);
        }
        break;
    case 3:
        if (motion_done(1000)) {
            lift_stop();
            set_motion_step(4);
        }
        break;
    case 4:
        return true;
    default:
        reset_motion();
        break;
    }

    return false;
}

static bool do_place(void)
{
    return do_place_with_down_time(1000);
}

static bool do_place_stack(void)
{
    return do_place_with_down_time(1000 + STACK_EXTRA_MS);
}

static bool do_tray_drop(uint8_t slot)
{
    /* 完整流程: 夹爪张开→下降→抓取→上升→旋转至托盘→下降→释放→上升→回位+转盘旋转 */
    switch (motion_step) {
    case 0:     /* 夹爪张开 */
        gripper_open();
        set_motion_step(1);
        break;
    case 1:     /* 下降 */
        if (motion_done(300)) {
            lift_move_down();
            set_motion_step(2);
        }
        break;
    case 2:     /* 夹爪抓取 */
        if (motion_done(1000)) {
            gripper_close();
            set_motion_step(3);
        }
        break;
    case 3:     /* 上升 */
        if (motion_done(500)) {
            lift_move_up();
            set_motion_step(4);
        }
        break;
    case 4:     /* 旋转夹爪至托盘上方 */
        if (motion_done(1000)) {
            lift_stop();
            gripper_rotate_to_tray();
            set_motion_step(5);
        }
        break;
    case 5:     /* 下降至托盘 */
        if (motion_done(300)) {
            lift_move_down();
            set_motion_step(6);
        }
        break;
    case 6:     /* 夹爪释放 */
        if (motion_done(1000)) {
            gripper_open();
            set_motion_step(7);
        }
        break;
    case 7:     /* 上升 */
        if (motion_done(500)) {
            lift_move_up();
            set_motion_step(8);
        }
        break;
    case 8:     /* 夹爪回位 + 托盘旋转120° */
        if (motion_done(1000)) {
            lift_stop();
            gripper_rotate_center();
            tray_rotate_to_slot(slot);
            set_motion_step(9);
        }
        break;
    case 9:     /* 等待旋转完成 */
        if (motion_done(300)) {
            return true;
        }
        break;
    default:
        reset_motion();
        break;
    }

    return false;
}

static bool do_tray_pick(uint8_t slot)
{
    switch (motion_step) {
    case 0:
        prepare_tray_slot(slot);
        gripper_open();
        set_motion_step(1);
        break;
    case 1:
        if (motion_done(300)) {
            lift_move_down();
            set_motion_step(2);
        }
        break;
    case 2:
        if (motion_done(1000)) {
            gripper_close();
            set_motion_step(3);
        }
        break;
    case 3:
        if (motion_done(500)) {
            lift_move_up();
            set_motion_step(4);
        }
        break;
    case 4:
        if (motion_done(1000)) {
            lift_stop();
            gripper_rotate_center();
            set_motion_step(5);
        }
        break;
    case 5:
        if (motion_done(300)) {
            return true;
        }
        break;
    default:
        reset_motion();
        break;
    }

    return false;
}

static void nav_goto_vision_offset(void)
{
    /* 视觉偏移是相机坐标系, 需旋转到全局坐标系 */
    float cos_a = cosf(pos_current.angle);
    float sin_a = sinf(pos_current.angle);
    float dx = cos_a * vision_current.offset_x - sin_a * vision_current.offset_y;
    float dy = sin_a * vision_current.offset_x + cos_a * vision_current.offset_y;
    nav_goto_heading(pos_current.x + dx, pos_current.y + dy, pos_current.angle);
}

static bool wait_reached_or_timeout(void)
{
    return nav_get_state() == NAV_REACHED || step_done(NAV_ALIGN_TIMEOUT_MS);
}

/* 检查 do_tray_pick 是否已完成夹取(夹爪已闭合, 正在提升) */
static bool tray_pick_gripper_closed(void)
{
    return motion_step >= 3;
}

void task_init(void)
{
    enter_state(TASK_INIT);
}

void task_update(void)
{
    nav_update();
    positioning_process();
    vision_process();
    qr_code_process();

    switch (task_state) {
    case TASK_INIT:
        gripper_open();
        tray_rotate_to_slot(0);
        gripper_rotate_center();
        lift_stop();
        enter_state(TASK_NAV_QR);
        break;

    case TASK_NAV_QR:
        /* 一键启动 → 等OPS9就绪 → 导航到二维码板搜索区域 */
        if (action_step == 0) {
            if (positioning_is_valid()) {
                nav_goto_heading(QR_SEARCH_X, QR_SEARCH_Y, QR_SEARCH_ANGLE);
                set_action_step(1);
            }
        }
        if (nav_get_state() == NAV_REACHED) {
            enter_state(TASK_WAIT_QR);
        }
        break;

    case TASK_WAIT_QR:
        /* 已到达二维码板附近, 等待OPS9定位有效 + QR码扫描完成 */
        if (positioning_is_valid() && qr_code_available()) {
            enter_state(TASK_B1_PICK_1);
        }
        break;

    case TASK_B1_PICK_1:
    case TASK_B1_PICK_2:
    case TASK_B1_PICK_3:
    {
        uint8_t idx = state_index3(task_state, TASK_B1_PICK_1, TASK_B1_PICK_2, TASK_B1_PICK_3);
        uint8_t color = qr_task.batch1_colors[idx];

        switch (action_step) {
        case 0:
            nav_goto_heading(RAW_MATERIAL_X, RAW_MATERIAL_Y, RAW_MATERIAL_FACE);
            vision_retry_cnt = 0;
            task_stats.pick_attempts++;
            set_action_step(1);
            break;
        case 1:
            if (nav_get_state() == NAV_REACHED) {
                vision_request_recognize();
                set_action_step(2);
            }
            break;
        case 2:
            if (vision_data_available() && vision_current.detected) {
                if (vision_current.barcode_id == color) {
                    nav_goto_vision_offset();
                    set_action_step(3);
                } else {
                    vision_request_recognize();
                }
            } else if (step_done(VISION_WAIT_TIMEOUT_MS)) {
                vision_retry_cnt++;
                if (vision_retry_cnt >= VISION_MAX_RETRIES) {
                    task_stats.vision_timeouts++;
                    vision_retry_cnt = 0;
                }
                vision_request_recognize();
                step_tick = HAL_GetTick();
            }
            break;
        case 3:
            if (wait_reached_or_timeout()) {
                set_action_step(4);
            }
            break;
        case 4:
            /* 最后一个物块: 夹爪闭合后立即导航到粗加工区, 与提升+转盘并行 */
            if (idx == 2 && motion_step >= 3 && !b1_nav_started) {
                float px, py;
                get_rough_pos(qr_task.batch1_positions[2], &px, &py);
                nav_goto_heading(px, py, ROUGH_FACE);
                b1_nav_started = true;
            }
            if (do_tray_drop(idx)) {
                b1_nav_started = false;
                task_stats.pick_successes++;
                task_state_t next[] = {TASK_B1_PICK_2, TASK_B1_PICK_3, TASK_B1_PLACE_R1};
                enter_state(next[idx]);
            }
            break;
        default:
            set_action_step(0);
            break;
        }
        break;
    }

    case TASK_B1_PLACE_R1:
    case TASK_B1_PLACE_R2:
    case TASK_B1_PLACE_R3:
    {
        uint8_t idx = state_index3(task_state, TASK_B1_PLACE_R1, TASK_B1_PLACE_R2, TASK_B1_PLACE_R3);
        uint8_t pos_num = qr_task.batch1_positions[idx];
        float px, py;
        get_rough_pos(pos_num, &px, &py);

        switch (action_step) {
        case 0:     /* 导航到粗加工区 */
            nav_goto_heading(px, py, ROUGH_FACE);
            set_action_step(1);
            break;
        case 1:     /* 到达 → 视觉定位 */
            if (nav_get_state() == NAV_REACHED) {
                vision_request_position();
                set_action_step(2);
            }
            break;
        case 2:     /* 视觉找到位置 → 偏移导航 */
            if (vision_data_available() && vision_current.detected) {
                nav_goto_vision_offset();
                set_action_step(3);
            } else if (step_done(NAV_ALIGN_TIMEOUT_MS)) {
                set_action_step(4);
            }
            break;
        case 3:     /* 到达精确位置 */
            if (wait_reached_or_timeout()) {
                set_action_step(4);
            }
            break;
        case 4:     /* 视觉定位完成 → 从托盘取物 */
            if (do_tray_pick(idx)) {
                set_action_step(5);
            } else if (idx == 2 && tray_pick_gripper_closed()) {
                /* 最后一个: 夹爪闭合后立即出发, 提升和导航并行 */
                nav_goto_heading(px, py, ROUGH_FACE);
                set_action_step(5);
            }
            break;
        case 5:     /* 放下物块 */
            if (do_place()) {
                task_stats.place_successes++;
                task_state_t next[] = {TASK_B1_PLACE_R2, TASK_B1_PLACE_R3, TASK_B1_MOVE_1};
                enter_state(next[idx]);
            }
            break;
        default:
            set_action_step(0);
            break;
        }
        break;
    }

    case TASK_B1_MOVE_1:
    case TASK_B1_MOVE_2:
    case TASK_B1_MOVE_3:
    {
        uint8_t idx = state_index3(task_state, TASK_B1_MOVE_1, TASK_B1_MOVE_2, TASK_B1_MOVE_3);
        float rx, ry, tx, ty;
        get_rough_pos(qr_task.batch1_positions[idx], &rx, &ry);
        get_temp_pos(qr_task.batch1_positions[idx], &tx, &ty);

        switch (action_step) {
        case 0:
            nav_goto_heading(rx, ry, ROUGH_FACE);
            set_action_step(1);
            break;
        case 1:
            if (nav_get_state() == NAV_REACHED) {
                vision_request_position();
                set_action_step(2);
            }
            break;
        case 2:
            if (vision_data_available() && vision_current.detected) {
                nav_goto_vision_offset();
                set_action_step(3);
            } else if (step_done(NAV_ALIGN_TIMEOUT_MS)) {
                set_action_step(4);
            }
            break;
        case 3:
            if (wait_reached_or_timeout()) {
                set_action_step(4);
            }
            break;
        case 4:
            if (do_pick()) {
                set_action_step(5);
            }
            break;
        case 5:
            if (do_tray_drop(idx)) {
                set_action_step(6);
            }
            break;
        case 6:
            nav_goto_heading(tx, ty, TEMP_FACE);
            set_action_step(7);
            break;
        case 7:
            if (nav_get_state() == NAV_REACHED) {
                vision_request_position();
                set_action_step(8);
            }
            break;
        case 8:
            if (vision_data_available() && vision_current.detected) {
                nav_goto_vision_offset();
                set_action_step(9);
            } else if (step_done(NAV_ALIGN_TIMEOUT_MS)) {
                set_action_step(10);
            }
            break;
        case 9:
            if (wait_reached_or_timeout()) {
                set_action_step(10);
            }
            break;
        case 10:
            if (do_tray_pick(idx)) {
                set_action_step(11);
            } else if (idx == 2 && tray_pick_gripper_closed()) {
                nav_goto_heading(tx, ty, TEMP_FACE);
                set_action_step(11);
            }
            break;
        case 11:
            if (do_place()) {
                task_stats.place_successes++;
                task_state_t next[] = {TASK_B1_MOVE_2, TASK_B1_MOVE_3, TASK_B2_PICK_1};
                enter_state(next[idx]);
            }
            break;
        default:
            set_action_step(0);
            break;
        }
        break;
    }

    case TASK_B2_PICK_1:
    case TASK_B2_PICK_2:
    case TASK_B2_PICK_3:
    {
        uint8_t idx = state_index3(task_state, TASK_B2_PICK_1, TASK_B2_PICK_2, TASK_B2_PICK_3);
        uint8_t color = qr_task.batch2_colors[idx];

        switch (action_step) {
        case 0:
            nav_goto_heading(RAW_MATERIAL_X, RAW_MATERIAL_Y, RAW_MATERIAL_FACE);
            vision_retry_cnt = 0;
            task_stats.pick_attempts++;
            set_action_step(1);
            break;
        case 1:
            if (nav_get_state() == NAV_REACHED) {
                vision_request_recognize();
                set_action_step(2);
            }
            break;
        case 2:
            if (vision_data_available() && vision_current.detected) {
                if (vision_current.barcode_id == color) {
                    nav_goto_vision_offset();
                    set_action_step(3);
                } else {
                    vision_request_recognize();
                }
            } else if (step_done(VISION_WAIT_TIMEOUT_MS)) {
                vision_retry_cnt++;
                if (vision_retry_cnt >= VISION_MAX_RETRIES) {
                    task_stats.vision_timeouts++;
                    vision_retry_cnt = 0;
                }
                vision_request_recognize();
                step_tick = HAL_GetTick();
            }
            break;
        case 3:
            if (wait_reached_or_timeout()) {
                set_action_step(4);
            }
            break;
        case 4:
            /* 最后一个物块: 夹爪闭合后立即导航到粗加工区, 与提升+转盘并行 */
            if (idx == 2 && motion_step >= 3 && !b2_nav_started) {
                float px, py;
                get_rough_pos(qr_task.batch2_positions[2], &px, &py);
                nav_goto_heading(px, py, ROUGH_FACE);
                b2_nav_started = true;
            }
            if (do_tray_drop(idx)) {
                b2_nav_started = false;
                task_stats.pick_successes++;
                task_state_t next[] = {TASK_B2_PICK_2, TASK_B2_PICK_3, TASK_B2_PLACE_R1};
                enter_state(next[idx]);
            }
            break;
        default:
            set_action_step(0);
            break;
        }
        break;
    }

    case TASK_B2_PLACE_R1:
    case TASK_B2_PLACE_R2:
    case TASK_B2_PLACE_R3:
    {
        uint8_t idx = state_index3(task_state, TASK_B2_PLACE_R1, TASK_B2_PLACE_R2, TASK_B2_PLACE_R3);
        uint8_t pos_num = qr_task.batch2_positions[idx];
        float px, py;
        get_rough_pos(pos_num, &px, &py);

        switch (action_step) {
        case 0:     /* 导航到粗加工区 */
            nav_goto_heading(px, py, ROUGH_FACE);
            set_action_step(1);
            break;
        case 1:     /* 到达 → 视觉定位 */
            if (nav_get_state() == NAV_REACHED) {
                vision_request_position();
                set_action_step(2);
            }
            break;
        case 2:     /* 视觉找到位置 → 偏移导航 */
            if (vision_data_available() && vision_current.detected) {
                nav_goto_vision_offset();
                set_action_step(3);
            } else if (step_done(NAV_ALIGN_TIMEOUT_MS)) {
                set_action_step(4);
            }
            break;
        case 3:     /* 到达精确位置 */
            if (wait_reached_or_timeout()) {
                set_action_step(4);
            }
            break;
        case 4:     /* 视觉定位完成 → 从托盘取物 */
            if (do_tray_pick(idx)) {
                set_action_step(5);
            } else if (idx == 2 && tray_pick_gripper_closed()) {
                nav_goto_heading(px, py, ROUGH_FACE);
                set_action_step(5);
            }
            break;
        case 5:     /* 放下物块 */
            if (do_place()) {
                task_stats.place_successes++;
                task_state_t next[] = {TASK_B2_PLACE_R2, TASK_B2_PLACE_R3, TASK_B2_RETURN_RAW};
                enter_state(next[idx]);
            }
            break;
        default:
            set_action_step(0);
            break;
        }
        break;
    }

    case TASK_B2_RETURN_RAW:
        /* 第二批粗加工区放完 → 返回原料区, 准备搬运到暂存区 */
        if (action_step == 0) {
            nav_goto_heading(RAW_MATERIAL_X, RAW_MATERIAL_Y, RAW_MATERIAL_FACE);
            set_action_step(1);
        }
        if (nav_get_state() == NAV_REACHED) {
            enter_state(TASK_B2_MOVE_1);
        }
        break;

    case TASK_B2_MOVE_1:
    case TASK_B2_MOVE_2:
    case TASK_B2_MOVE_3:
    {
        uint8_t idx = state_index3(task_state, TASK_B2_MOVE_1, TASK_B2_MOVE_2, TASK_B2_MOVE_3);
        float rx, ry, tx, ty;
        get_rough_pos(qr_task.batch2_positions[idx], &rx, &ry);
        get_temp_pos(qr_task.batch1_positions[idx], &tx, &ty);  /* 暂存区位置同第一批, 保证堆叠颜色一致 */

        switch (action_step) {
        case 0:
            nav_goto_heading(rx, ry, ROUGH_FACE);
            set_action_step(1);
            break;
        case 1:
            if (nav_get_state() == NAV_REACHED) {
                vision_request_position();
                set_action_step(2);
            }
            break;
        case 2:
            if (vision_data_available() && vision_current.detected) {
                nav_goto_vision_offset();
                set_action_step(3);
            } else if (step_done(NAV_ALIGN_TIMEOUT_MS)) {
                set_action_step(4);
            }
            break;
        case 3:
            if (wait_reached_or_timeout()) {
                set_action_step(4);
            }
            break;
        case 4:
            if (do_pick()) {
                set_action_step(5);
            }
            break;
        case 5:
            if (do_tray_drop(idx)) {
                set_action_step(6);
            }
            break;
        case 6:
            nav_goto_heading(tx, ty, TEMP_FACE);
            set_action_step(7);
            break;
        case 7:
            if (nav_get_state() == NAV_REACHED) {
                vision_request_position();
                set_action_step(8);
            }
            break;
        case 8:
            if (vision_data_available() && vision_current.detected) {
                nav_goto_vision_offset();
                set_action_step(9);
            } else if (step_done(NAV_ALIGN_TIMEOUT_MS)) {
                set_action_step(10);
            }
            break;
        case 9:
            if (wait_reached_or_timeout()) {
                set_action_step(10);
            }
            break;
        case 10:
            if (do_tray_pick(idx)) {
                set_action_step(11);
            } else if (idx == 2 && tray_pick_gripper_closed()) {
                nav_goto_heading(tx, ty, TEMP_FACE);
                set_action_step(11);
            }
            break;
        case 11:
            if (do_place_stack()) {
                task_stats.place_successes++;
                task_state_t next[] = {TASK_B2_MOVE_2, TASK_B2_MOVE_3, TASK_RETURN_HOME};
                enter_state(next[idx]);
            }
            break;
        default:
            set_action_step(0);
            break;
        }
        break;
    }

    case TASK_RETURN_HOME:
        if (action_step == 0) {
            nav_goto_heading(START_X, START_Y, START_ANGLE);
            set_action_step(1);
        }
        if (nav_get_state() == NAV_REACHED) {
            enter_state(TASK_DONE);
        }
        break;

    case TASK_DONE:
        chassis_stop();
        {
            /* 一次性发送统计信息到串口屏 */
            static bool stats_sent = false;
            if (!stats_sent) {
                char buf[80];
                int len = snprintf(buf, sizeof(buf),
                    "PICK:%d/%d PLACE:%d VT:%d\r\n",
                    task_stats.pick_successes,
                    task_stats.pick_attempts,
                    task_stats.place_successes,
                    task_stats.vision_timeouts);
                HAL_UART_Transmit(&huart4, (uint8_t *)buf, len, 100);
                stats_sent = true;
            }
        }
        break;

    default:
        chassis_stop();
        enter_state(TASK_DONE);
        break;
    }
}

task_state_t task_get_state(void)
{
    return task_state;
}
