#include "task.h"
#include <stdio.h>

#define NAV_ALIGN_TIMEOUT_MS   2000U
#define VISION_WAIT_TIMEOUT_MS 3000U
#define VISION_MAX_RETRIES     3U      /* 视觉识别最大重试次数 */
#define STATE_TIMEOUT_DEFAULT  15000U  /* 默认状态超时 15秒 */

task_state_t task_state = TASK_INIT;

static uint32_t state_enter_tick = 0;
static uint8_t  action_step = 0;
static uint32_t step_tick = 0;

static uint8_t  motion_step = 0;
static uint32_t motion_tick = 0;

static uint8_t  vision_retry_cnt = 0;  /* 视觉重试计数 */
static bool     batch_nav_started[2] = {false, false}; /* B1/B2并行导航标志 */

/* 抓取统计 */
task_stats_t task_stats = {0};

/* 错误恢复相关 */
static error_code_t  last_error = ERR_NONE;
static task_state_t  saved_error_state = TASK_INIT;   /* 出错前的状态 */
static uint8_t       saved_vision_skip_step = 0;      /* 跳过视觉时的action_step */

/* ========== 状态超时配置表 ========== */
static const state_timeout_t state_timeouts[] = {
    { TASK_INIT,        5000  },
    { TASK_NAV_QR,      15000 },
    { TASK_WAIT_QR,     10000 },
    { TASK_B1_PICK_1,   20000 },
    { TASK_B1_PICK_2,   20000 },
    { TASK_B1_PICK_3,   20000 },
    { TASK_B1_PLACE_R1, 20000 },
    { TASK_B1_PLACE_R2, 20000 },
    { TASK_B1_PLACE_R3, 20000 },
    { TASK_B1_MOVE_1,   30000 },
    { TASK_B1_MOVE_2,   30000 },
    { TASK_B1_MOVE_3,   30000 },
    { TASK_B2_PICK_1,   20000 },
    { TASK_B2_PICK_2,   20000 },
    { TASK_B2_PICK_3,   20000 },
    { TASK_B2_PLACE_R1, 20000 },
    { TASK_B2_PLACE_R2, 20000 },
    { TASK_B2_PLACE_R3, 20000 },
    { TASK_B2_RETURN_RAW, 15000 },
    { TASK_B2_MOVE_1,   30000 },
    { TASK_B2_MOVE_2,   30000 },
    { TASK_B2_MOVE_3,   30000 },
    { TASK_RETURN_HOME, 15000 },
    { TASK_DONE,        0     },  /* 完成状态不超时 */
    { TASK_ERROR,       0     },  /* 错误状态不超时 */
};

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

/* ========== 状态超时检查 ========== */
static uint32_t get_state_timeout(task_state_t state)
{
    for (uint32_t i = 0; i < sizeof(state_timeouts) / sizeof(state_timeouts[0]); i++) {
        if (state_timeouts[i].state == state) return state_timeouts[i].timeout_ms;
    }
    return STATE_TIMEOUT_DEFAULT;
}

static bool state_timed_out(void)
{
    uint32_t timeout = get_state_timeout(task_state);
    if (timeout == 0) return false;
    return (HAL_GetTick() - state_enter_tick) > timeout;
}

error_code_t task_get_last_error(void)
{
    return last_error;
}

static bool do_pick(void)
{
    switch (motion_step) {
    case 0:
        lift_move_down();
        set_motion_step(1);
        break;
    case 1:
        if (motion_done(500)) {
            gripper_close();
            set_motion_step(2);
        }
        break;
    case 2:
        if (motion_done(300)) {
            lift_move_up();
            set_motion_step(3);
        }
        break;
    case 3:
        if (motion_done(500)) {
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
        if (motion_done(300)) {
            lift_move_up();
            set_motion_step(3);
        }
        break;
    case 3:
        if (motion_done(500)) {
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
    return do_place_with_down_time(500);
}

static bool do_place_stack(void)
{
    return do_place_with_down_time(500 + STACK_EXTRA_MS);
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
        if (motion_done(100)) {
            lift_move_down();
            set_motion_step(2);
        }
        break;
    case 2:     /* 夹爪抓取 */
        if (motion_done(500)) {
            gripper_close();
            set_motion_step(3);
        }
        break;
    case 3:     /* 上升 */
        if (motion_done(300)) {
            lift_move_up();
            set_motion_step(4);
        }
        break;
    case 4:     /* 旋转夹爪至托盘上方 */
        if (motion_done(500)) {
            lift_stop();
            gripper_rotate_to_tray();
            set_motion_step(5);
        }
        break;
    case 5:     /* 下降至托盘 */
        if (motion_done(100)) {
            lift_move_down();
            set_motion_step(6);
        }
        break;
    case 6:     /* 夹爪释放 */
        if (motion_done(500)) {
            gripper_open();
            set_motion_step(7);
        }
        break;
    case 7:     /* 上升 */
        if (motion_done(300)) {
            lift_move_up();
            set_motion_step(8);
        }
        break;
    case 8:     /* 夹爪回位 + 托盘旋转120° */
        if (motion_done(500)) {
            lift_stop();
            gripper_rotate_center();
            tray_rotate_to_slot(slot);
            set_motion_step(9);
        }
        break;
    case 9:     /* 等待旋转完成 */
        if (motion_done(100)) {
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
        if (motion_done(100)) {
            lift_move_down();
            set_motion_step(2);
        }
        break;
    case 2:
        if (motion_done(500)) {
            gripper_close();
            set_motion_step(3);
        }
        break;
    case 3:
        if (motion_done(300)) {
            lift_move_up();
            set_motion_step(4);
        }
        break;
    case 4:
        if (motion_done(500)) {
            lift_stop();
            gripper_rotate_center();
            set_motion_step(5);
        }
        break;
    case 5:
        if (motion_done(100)) {
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
    return nav_get_state() == NAV_REACHED || nav_get_state() == NAV_FAILED
           || step_done(NAV_ALIGN_TIMEOUT_MS);
}

/* 检查导航是否失败, 若失败则进入错误状态, 返回true表示已处理 */
static bool check_nav_failed(void)
{
    if (nav_get_state() == NAV_FAILED) {
        saved_error_state = task_state;
        last_error = nav_get_error();
        enter_state(TASK_ERROR);
        return true;
    }
    return false;
}

/* 检查 do_tray_pick 是否已完成夹取(夹爪已闭合, 正在提升) */
static bool tray_pick_gripper_closed(void)
{
    return motion_step >= 3;
}

/* ========== 批次公共流程上下文 ========== */
typedef struct {
    const uint8_t       *colors;       /* 批次颜色数组 */
    const uint8_t       *positions;    /* 批次位置数组 */
    bool                *nav_started;  /* 并行导航标志指针 */
    const task_state_t  *next_states;  /* 后继状态 [pick2, pick3, place_r1] */
} batch_pick_ctx_t;

typedef struct {
    const uint8_t       *positions;    /* 批次位置数组 */
    const task_state_t  *next_states;  /* 后继状态 [place_r2, place_r3, next_phase] */
} batch_place_ctx_t;

typedef struct {
    const uint8_t       *positions;     /* 粗加工区位置数组 */
    const uint8_t       *temp_positions;/* 暂存区位置数组 (B2用B1的位置保证堆叠) */
    bool                *nav_started;   /* 并行导航标志指针 */
    const task_state_t  *next_states;   /* 后继状态 [move_2, move_3, next_phase] */
    bool                 use_stack;     /* 是否使用堆叠放置 */
} batch_move_ctx_t;

/* ========== 公共流程: 原料区取料 ========== */
static bool do_batch_pick(uint8_t idx, const batch_pick_ctx_t *ctx)
{
    uint8_t color = ctx->colors[idx];

    switch (action_step) {
    case 0:
        nav_goto_heading(RAW_MATERIAL_X, RAW_MATERIAL_Y, RAW_MATERIAL_FACE);
        vision_retry_cnt = 0;
        task_stats.pick_attempts++;
        set_action_step(1);
        break;
    case 1:
        if (check_nav_failed()) break;
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
                saved_error_state = task_state;
                saved_vision_skip_step = 4;
                last_error = ERR_VISION_MAX_RETRIES;
                enter_state(TASK_ERROR);
                return false;
            }
            vision_request_recognize();
            step_tick = HAL_GetTick();
        }
        break;
    case 3:
        if (check_nav_failed()) break;
        if (wait_reached_or_timeout()) set_action_step(4);
        break;
    case 4:
        if (idx == 2 && motion_step >= 3 && !*(ctx->nav_started)) {
            float px, py;
            get_rough_pos(ctx->positions[2], &px, &py);
            nav_goto_heading(px, py, ROUGH_FACE);
            *(ctx->nav_started) = true;
        }
        if (do_tray_drop(idx)) {
            *(ctx->nav_started) = false;
            task_stats.pick_successes++;
            enter_state(ctx->next_states[idx]);
        }
        break;
    default:
        set_action_step(0);
        break;
    }
    return true;
}

/* ========== 公共流程: 粗加工区放置 ========== */
static bool do_batch_place(uint8_t idx, const batch_place_ctx_t *ctx)
{
    float px, py;
    get_rough_pos(ctx->positions[idx], &px, &py);

    switch (action_step) {
    case 0:
        nav_goto_heading(px, py, ROUGH_FACE);
        set_action_step(1);
        break;
    case 1:
        if (check_nav_failed()) break;
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
        if (check_nav_failed()) break;
        if (wait_reached_or_timeout()) set_action_step(4);
        break;
    case 4:
        if (do_tray_pick(idx)) {
            set_action_step(5);
        } else if (idx == 2 && tray_pick_gripper_closed()) {
            nav_goto_heading(px, py, ROUGH_FACE);
            set_action_step(5);
        }
        break;
    case 5:
        if (do_place()) {
            task_stats.place_successes++;
            enter_state(ctx->next_states[idx]);
        }
        break;
    default:
        set_action_step(0);
        break;
    }
    return true;
}

/* ========== 公共流程: 粗加工区→暂存区搬运 ========== */
static bool do_batch_move(uint8_t idx, const batch_move_ctx_t *ctx)
{
    float rx, ry, tx, ty;
    get_rough_pos(ctx->positions[idx], &rx, &ry);
    get_temp_pos(ctx->temp_positions[idx], &tx, &ty);

    switch (action_step) {
    case 0:
        nav_goto_heading(rx, ry, ROUGH_FACE);
        set_action_step(1);
        break;
    case 1:
        if (check_nav_failed()) break;
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
        if (check_nav_failed()) break;
        if (wait_reached_or_timeout()) set_action_step(4);
        break;
    case 4:
        if (do_pick()) set_action_step(5);
        break;
    case 5:
        if (do_tray_drop(idx)) {
            *(ctx->nav_started) = false;
            set_action_step(7);
        } else if (motion_step == 7 && !*(ctx->nav_started)) {
            nav_goto_heading(tx, ty, TEMP_FACE);
            *(ctx->nav_started) = true;
        }
        break;
    case 6:
        break; /* 已被并行导航跳过 */
    case 7:
        if (check_nav_failed()) break;
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
        if (check_nav_failed()) break;
        if (wait_reached_or_timeout()) set_action_step(10);
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
        if (ctx->use_stack ? do_place_stack() : do_place()) {
            task_stats.place_successes++;
            enter_state(ctx->next_states[idx]);
        }
        break;
    default:
        set_action_step(0);
        break;
    }
    return true;
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

    /* 全局状态超时检查 (跳过 INIT/DONE/ERROR 状态) */
    if (task_state != TASK_INIT && task_state != TASK_DONE && task_state != TASK_ERROR) {
        if (state_timed_out()) {
            saved_error_state = task_state;
            last_error = ERR_STATE_TIMEOUT;
            enter_state(TASK_ERROR);
            return;
        }
    }

    switch (task_state) {
    case TASK_INIT:
        gripper_open();
        gripper_rotate_center();
        tray_rotate_to_slot(0);
        lift_move_up();
        if (motion_done(1500)) {
            lift_stop();
            enter_state(TASK_NAV_QR);
        }
        break;

    case TASK_NAV_QR:
        /* 一键启动 → 等OPS9就绪 → 导航到二维码板搜索区域 */
        if (action_step == 0) {
            if (positioning_is_valid()) {
                nav_goto_heading(QR_SEARCH_X, QR_SEARCH_Y, QR_SEARCH_ANGLE);
                set_action_step(1);
            }
        }
        if (check_nav_failed()) break;
        if (nav_get_state() == NAV_REACHED) {
            enter_state(TASK_WAIT_QR);
        }
        break;

    case TASK_WAIT_QR:
        /* 已到达二维码板附近, 等待OPS9定位有效 + QR码扫描完成 */
        if (positioning_is_valid() && qr_code_available()) {
            enter_state(TASK_B1_PICK_1);
        } else if (step_done(10000)) {
            /* 10秒无QR码 → 超时错误 */
            saved_error_state = task_state;
            last_error = ERR_QR_TIMEOUT;
            enter_state(TASK_ERROR);
        }
        break;

    case TASK_B1_PICK_1:
    case TASK_B1_PICK_2:
    case TASK_B1_PICK_3:
    {
        static const task_state_t next[] = {TASK_B1_PICK_2, TASK_B1_PICK_3, TASK_B1_PLACE_R1};
        batch_pick_ctx_t ctx = {
            .colors = qr_task.batch1_colors,
            .positions = qr_task.batch1_positions,
            .nav_started = &batch_nav_started[0],
            .next_states = next,
        };
        do_batch_pick(state_index3(task_state, TASK_B1_PICK_1, TASK_B1_PICK_2, TASK_B1_PICK_3), &ctx);
        break;
    }

    case TASK_B1_PLACE_R1:
    case TASK_B1_PLACE_R2:
    case TASK_B1_PLACE_R3:
    {
        static const task_state_t next[] = {TASK_B1_PLACE_R2, TASK_B1_PLACE_R3, TASK_B1_MOVE_1};
        batch_place_ctx_t ctx = {
            .positions = qr_task.batch1_positions,
            .next_states = next,
        };
        do_batch_place(state_index3(task_state, TASK_B1_PLACE_R1, TASK_B1_PLACE_R2, TASK_B1_PLACE_R3), &ctx);
        break;
    }

    case TASK_B1_MOVE_1:
    case TASK_B1_MOVE_2:
    case TASK_B1_MOVE_3:
    {
        static const task_state_t next[] = {TASK_B1_MOVE_2, TASK_B1_MOVE_3, TASK_B2_PICK_1};
        batch_move_ctx_t ctx = {
            .positions = qr_task.batch1_positions,
            .temp_positions = qr_task.batch1_positions,
            .nav_started = &batch_nav_started[0],
            .next_states = next,
            .use_stack = false,
        };
        do_batch_move(state_index3(task_state, TASK_B1_MOVE_1, TASK_B1_MOVE_2, TASK_B1_MOVE_3), &ctx);
        break;
    }

    case TASK_B2_PICK_1:
    case TASK_B2_PICK_2:
    case TASK_B2_PICK_3:
    {
        static const task_state_t next[] = {TASK_B2_PICK_2, TASK_B2_PICK_3, TASK_B2_PLACE_R1};
        batch_pick_ctx_t ctx = {
            .colors = qr_task.batch2_colors,
            .positions = qr_task.batch2_positions,
            .nav_started = &batch_nav_started[1],
            .next_states = next,
        };
        do_batch_pick(state_index3(task_state, TASK_B2_PICK_1, TASK_B2_PICK_2, TASK_B2_PICK_3), &ctx);
        break;
    }

    case TASK_B2_PLACE_R1:
    case TASK_B2_PLACE_R2:
    case TASK_B2_PLACE_R3:
    {
        static const task_state_t next[] = {TASK_B2_PLACE_R2, TASK_B2_PLACE_R3, TASK_B2_RETURN_RAW};
        batch_place_ctx_t ctx = {
            .positions = qr_task.batch2_positions,
            .next_states = next,
        };
        do_batch_place(state_index3(task_state, TASK_B2_PLACE_R1, TASK_B2_PLACE_R2, TASK_B2_PLACE_R3), &ctx);
        break;
    }

    case TASK_B2_RETURN_RAW:
        /* 第二批粗加工区放完 → 返回原料区, 准备搬运到暂存区 */
        if (action_step == 0) {
            nav_goto_heading(RAW_MATERIAL_X, RAW_MATERIAL_Y, RAW_MATERIAL_FACE);
            set_action_step(1);
        }
        if (check_nav_failed()) break;
        if (nav_get_state() == NAV_REACHED) {
            enter_state(TASK_B2_MOVE_1);
        }
        break;

    case TASK_B2_MOVE_1:
    case TASK_B2_MOVE_2:
    case TASK_B2_MOVE_3:
    {
        static const task_state_t next[] = {TASK_B2_MOVE_2, TASK_B2_MOVE_3, TASK_RETURN_HOME};
        batch_move_ctx_t ctx = {
            .positions = qr_task.batch2_positions,
            .temp_positions = qr_task.batch1_positions,  /* 暂存区同B1, 保证堆叠 */
            .nav_started = &batch_nav_started[1],
            .next_states = next,
            .use_stack = true,
        };
        do_batch_move(state_index3(task_state, TASK_B2_MOVE_1, TASK_B2_MOVE_2, TASK_B2_MOVE_3), &ctx);
        break;
    }

    case TASK_RETURN_HOME:
        if (action_step == 0) {
            nav_goto_heading(START_X, START_Y, START_ANGLE);
            set_action_step(1);
        }
        if (check_nav_failed()) break;
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

    case TASK_ERROR:
    {
        chassis_stop();
        nav_stop();

        static uint32_t error_entry_tick = 0;
        static bool error_logged = false;

        if (!error_logged) {
            error_logged = true;
            error_entry_tick = HAL_GetTick();
        }

        /* 根据错误类型决定恢复策略 */
        recovery_action_t action = RECOVERY_ABORT_TASK;
        switch (last_error) {
        case ERR_NAV_TIMEOUT:
        case ERR_NAV_POS_LOST:
            action = RECOVERY_RETRY;
            break;
        case ERR_VISION_TIMEOUT:
        case ERR_VISION_MAX_RETRIES:
            action = RECOVERY_SKIP_VISION;
            break;
        case ERR_QR_TIMEOUT:
        case ERR_OBSTACLE_STUCK:
            action = RECOVERY_ABORT_TASK;
            break;
        default:
            action = RECOVERY_RETRY;
            break;
        }

        /* 等待1秒后执行恢复 */
        if (HAL_GetTick() - error_entry_tick > 1000) {
            error_logged = false;
            last_error = ERR_NONE;
            switch (action) {
            case RECOVERY_RETRY:
                enter_state(saved_error_state);
                break;
            case RECOVERY_SKIP_VISION:
                task_state = saved_error_state;
                action_step = saved_vision_skip_step;
                state_enter_tick = HAL_GetTick();
                step_tick = HAL_GetTick();
                reset_motion();
                break;
            case RECOVERY_ABORT_TASK:
            default:
                enter_state(TASK_DONE);
                break;
            }
        }
        break;
    }

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
