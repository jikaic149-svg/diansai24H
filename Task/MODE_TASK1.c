/**
 * @file  MODE_TASK1.c
 * @brief 任务1：直线冲刺，遇到黑线立刻停车
 *        轮速已闭环版：遇线后输出 0 目标速度，编码器检测停稳
 *
 * 注意：Task 内部不得调用 Mode_SetXxx()，否则会覆盖顶层模式！
 *       直接调用底层 MODE_Xxx_Init/Exec 函数。
 */
#include "MODE_TASK1.h"
#include "Mode/MODE_STRAIGHT.h"
#include "Application/broker/broker.h"
#include "Device/track/track.h"
#include "config.h"
#include <math.h>

/* ======================== 内部状态 ======================== */
typedef enum {
    T1_STEP_STRAIGHT = 0,   /* 直线冲刺 */
    T1_STEP_BRAKE,          /* 速度闭环制动 (目标=0, PID主动刹车) */
    T1_STEP_DONE            /* 完成 */
} Task1Step_e;

static Task1Step_e s_step       = T1_STEP_STRAIGHT;
static uint8_t     s_done       = 0;
static float       s_lock_yaw   = 0.0f;
static uint16_t    s_tick       = 0;
static uint8_t     s_stable_cnt = 0;

/* ======================== 公共接口 ======================== */

void Task1_Init(void)
{
    s_step       = T1_STEP_STRAIGHT;
    s_done       = 0;
    s_tick       = 0;
    s_stable_cnt = 0;
    s_lock_yaw   = Broker_Subscribe_Float(TOPIC_JY61P_YAW);

    /* 直接初始化底层直线模块，不调用 Mode_SetStraight! */
    MODE_Straight_Init();
}

void Task1_Exec(void)
{
    if (s_done) return;

    float track_err = Broker_Subscribe_Float(TOPIC_TRACK_ERROR);
    float yaw       = Broker_Subscribe_Float(TOPIC_JY61P_YAW);

    switch (s_step)
    {
        /* ---- 直线冲刺：检测到黑线立刻制动 ---- */
        case T1_STEP_STRAIGHT:
        {
            s_tick++;

            /* 直接调用底层直线 Exec */
            MODE_Straight_Exec(TASK_STRAIGHT_FAST, s_lock_yaw, yaw);

            /* 前 5 个周期(~25ms@5ms) 不检测，避免起步误触发 */
            if (s_tick > 5 && track_err != TRACK_LOST_LINE) {
                /* 输出 0 目标速度，由速度闭环 PID 主动制动 */
                Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT,  0.0f);
                Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
                s_step       = T1_STEP_BRAKE;
                s_tick       = 0;
                s_stable_cnt = 0;
            }
            break;
        }

        /* ---- 速度闭环制动：PID自动刹车，编码器检测停稳 ---- */
        case T1_STEP_BRAKE:
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
                s_step = T1_STEP_DONE;
                s_done = 1;
            }
            break;
        }

        default:
            break;
    }
}

uint8_t Task1_IsDone(void)
{
    return s_done;
}
