/**
 * @file  MODE_TASK4.c
 * @brief 任务4：4圈 figure-8
 *   - 每圈都做 A 点对齐（防位置累积漂移）
 *   - 中间圈容忍度 3°（快速对齐）
 *   - 最后一圈容忍度 1°（精对齐）
 *   - 全程保持第一圈的 base_yaw
 */
#include "MODE_TASK4.h"
#include "MODE_TASK3.h"
#include "Application/broker/broker.h"
#include "config.h"
#include <stdint.h>

#define TASK4_TOTAL_LAPS  4
#define TASK4_COARSE_TOL  3.0f   /* 中间圈容忍度 */
#define TASK4_FINE_TOL    1.0f   /* 最后一圈容忍度 */

static uint8_t s_lap_count    = 0;
static uint8_t s_done         = 0;
static float   s_original_yaw = 0.0f;

void Task4_Init(void)
{
    s_lap_count    = 0;
    s_done         = 0;
    s_original_yaw = Broker_Subscribe_Float(TOPIC_JY61P_YAW);

    Task3_Init();
    Task3_SetAlignTol(TASK4_COARSE_TOL);
}

void Task4_Exec(void)
{
    if (s_done) return;

    Task3_Exec();

    if (Task3_IsDone()) {
        s_lap_count++;

        if (s_lap_count >= TASK4_TOTAL_LAPS) {
            s_done = 1;
        } else {
            Task3_Init();
            Task3_SetBaseYaw(s_original_yaw);

            if (s_lap_count >= TASK4_TOTAL_LAPS - 1) {
                Task3_SetAlignTol(TASK4_FINE_TOL);
            } else {
                Task3_SetAlignTol(TASK4_COARSE_TOL);
            }
        }
    }
}

uint8_t Task4_IsDone(void)
{
    return s_done;
}
