/**
 * @file  MODE_TASK2.c
 * @brief 任务2：A→B直线 + B→C循迹半圆 + C弧线转180° + C→D直线 + D→A循迹半圆
 *        三轮车适配版：直接调用底层 MODE 函数，不调用 Mode_SetXxx 避免模式覆盖
 */
#include "MODE_TASK2.h"
#include "Mode/MODE_STRAIGHT.h"
#include "Mode/MODE_TRACKING.h"
#include "Mode/MODE_CURVE.h"
#include "Application/broker/broker.h"
#include "Device/track/track.h"
#include "config.h"
#include <math.h>

/* ======================== 内部状态 ======================== */
typedef enum {
    T2_AB_STRAIGHT = 0,    /* A→B 直线冲刺 */
    T2_BC_TRACKING,        /* B→C 循迹半圆 */
    T2_C_CURVE_ALIGN,      /* C 点弧线转向对齐 180° */
    T2_CD_STRAIGHT,        /* C→D 直线冲刺 */
    T2_DA_TRACKING,        /* D→A 循迹半圆 */
    T2_BRAKE,              /* 终点制动 */
    T2_DONE                /* 完成 */
} Task2Step_e;

static Task2Step_e s_step         = T2_AB_STRAIGHT;
static uint8_t     s_done         = 0;
static float       s_lock_yaw     = 0.0f;
static float       s_target_yaw   = 0.0f;
static uint16_t    s_tick         = 0;
static uint16_t    s_track_valid  = 0;
static uint16_t    s_track_lost   = 0;
static uint8_t     s_stable_cnt   = 0;

static void _reset_track_cnt(void)
{
    s_track_valid = 0;
    s_track_lost  = 0;
}

/* ======================== 公共接口 ======================== */

void Task2_Init(void)
{
    s_step = T2_AB_STRAIGHT;
    s_done = 0;
    s_tick = 0;
    s_stable_cnt = 0;
    _reset_track_cnt();

    s_lock_yaw   = Broker_Subscribe_Float(TOPIC_JY61P_YAW);
    s_target_yaw = s_lock_yaw;

    /* 直接初始化底层，不调用 Mode_SetStraight! */
    MODE_Straight_Init();
}

void Task2_Exec(void)
{
    if (s_done) return;

    float yaw       = Broker_Subscribe_Float(TOPIC_JY61P_YAW);
    float track_err = Broker_Subscribe_Float(TOPIC_TRACK_ERROR);

    switch (s_step)
    {
        /* ==================== A→B 直线：遇线立刻循迹 ==================== */
        case T2_AB_STRAIGHT:
        {
            s_tick++;
            MODE_Straight_Exec(TASK_STRAIGHT_FAST, s_lock_yaw, yaw);

            /* 5 tick 盲区(~25ms) 后，一遇黑线立刻切循迹 */
            if (s_tick > 5 && track_err != TRACK_LOST_LINE) {
                MODE_Tracking_Init();
                _reset_track_cnt();
                s_tick = 0;
                s_step = T2_BC_TRACKING;
            }
            break;
        }

        /* ==================== B→C 循迹半圆 ==================== */
        case T2_BC_TRACKING:
        {
            MODE_Tracking_Exec(TASK_TRACK_HIGH, track_err);

            if (track_err != TRACK_LOST_LINE) {
                s_track_lost = 0;
                if (s_track_valid < 60000) s_track_valid++;
            } else {
                s_track_lost++;
            }

            /* 半圆结束判定：循迹足够久后连续丢线 */
            if ((s_track_valid > 80 && s_track_lost > 25) || s_track_lost > 120) {
                s_target_yaw += 180.0f;
                MODE_Curve_Init();
                s_tick = 0;
                s_step = T2_C_CURVE_ALIGN;
            }
            break;
        }

        /* ==================== C 点弧线对齐 180° ==================== */
        case T2_C_CURVE_ALIGN:
        {
            MODE_Curve_Exec(MODE_DEFAULT_CURVE_SPEED, s_target_yaw, yaw);

            float yaw_err = s_target_yaw - yaw;
            while (yaw_err >  180.0f) yaw_err -= 360.0f;
            while (yaw_err < -180.0f) yaw_err += 360.0f;

            s_tick++;

            /* 行进中遇到黑线 → 直接循迹 */
            if (s_tick > 5 && track_err != TRACK_LOST_LINE) {
                MODE_Tracking_Init();
                _reset_track_cnt();
                s_tick = 0;
                s_step = T2_DA_TRACKING;
                break;
            }

            /* 航向到位 or 超时 → 切直线找下一段线 */
            if (fabsf(yaw_err) < 5.0f || s_tick > 200) {
                MODE_Straight_Init();
                s_tick = 0;
                s_step = T2_CD_STRAIGHT;
            }
            break;
        }

        /* ==================== C→D 直线：遇线立刻循迹 ==================== */
        case T2_CD_STRAIGHT:
        {
            s_tick++;
            MODE_Straight_Exec(TASK_STRAIGHT_FAST, s_target_yaw, yaw);

            if (s_tick > 5 && track_err != TRACK_LOST_LINE) {
                MODE_Tracking_Init();
                _reset_track_cnt();
                s_tick = 0;
                s_step = T2_DA_TRACKING;
            }
            break;
        }

        /* ==================== D→A 循迹半圆 ==================== */
        case T2_DA_TRACKING:
        {
            MODE_Tracking_Exec(TASK_TRACK_HIGH, track_err);

            if (track_err != TRACK_LOST_LINE) {
                s_track_lost = 0;
                if (s_track_valid < 60000) s_track_valid++;
            } else {
                s_track_lost++;
            }

            /* 半圆结束 → 制动停车 */
            if ((s_track_valid > 80 && s_track_lost > 25) || s_track_lost > 120) {
                Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT,  0.0f);
                Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
                s_tick = 0;
                s_step = T2_BRAKE;
            }
            break;
        }

        /* ==================== 终点制动：编码器检测停稳 ==================== */
        case T2_BRAKE:
        {
            Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT,  0.0f);
            Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
            s_tick++;

            float spd_l = Broker_Subscribe_Float(TOPIC_SPEED_LEFT);
            float spd_r = Broker_Subscribe_Float(TOPIC_SPEED_RIGHT);
            if (fabsf(spd_l) < BRAKE_SPEED_THRESHOLD && fabsf(spd_r) < BRAKE_SPEED_THRESHOLD) {
                s_stable_cnt++;
            } else {
                s_stable_cnt = 0;
            }

            if (s_stable_cnt >= BRAKE_STABLE_COUNT || s_tick > 100) {
                s_step = T2_DONE;
                s_done = 1;
            }
            break;
        }

        default:
            break;
    }
}

uint8_t Task2_IsDone(void)
{
    return s_done;
}
