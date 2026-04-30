/**
 * @file  mode_manager.c
 * @brief 小车行为状态机管理器实现
 *        纯软逻辑，不含任何 MCU 头文件，所有数据交互走 Broker
 */
#include "mode_manager.h"
#include "MODE_IDLE.h"
#include "MODE_STRAIGHT.h"
#include "MODE_TRACKING.h"
#include "MODE_TURNING.h"
#include "MODE_CURVE.h"
#include "MODE_BRAKE.h"
#include "MODE_DONE.h"
#include "Task/MODE_TASK1.h"
#include "Task/MODE_TASK2.h"
#include "Task/MODE_TASK3.h"
#include "Task/MODE_TASK4.h"
#include "Application/broker/broker.h"

/* ======================== 模式上下文 ======================== */
typedef struct {
    CarMode_e mode;
    float base_pwm;
    float target_yaw;
    float max_pwm;
} ModeContext_t;

static ModeContext_t s_ctx = {
    .mode       = CAR_MODE_IDLE,
    .base_pwm   = 0.0f,
    .target_yaw = 0.0f,
    .max_pwm    = 0.0f
};

/* ======================== 公共 API ======================== */

void Mode_Init(void)
{
    s_ctx.mode = CAR_MODE_IDLE;
    MODE_Idle_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_IDLE);
}

void Mode_Exec(void)
{
    float yaw   = Broker_Subscribe_Float(TOPIC_JY61P_YAW);
    float track = Broker_Subscribe_Float(TOPIC_TRACK_ERROR);

    switch (s_ctx.mode)
    {
        case CAR_MODE_IDLE:
            MODE_Idle_Exec();
            break;

        case CAR_MODE_STRAIGHT:
            MODE_Straight_Exec(s_ctx.base_pwm, s_ctx.target_yaw, yaw);
            break;

        case CAR_MODE_TRACKING:
            MODE_Tracking_Exec(s_ctx.max_pwm, track);
            break;

        case CAR_MODE_TURNING:
            MODE_Turning_Exec(s_ctx.target_yaw, yaw);
            break;

        case CAR_MODE_CURVE:
            MODE_Curve_Exec(s_ctx.base_pwm, s_ctx.target_yaw, yaw);
            break;

        case CAR_MODE_BRAKE:
            MODE_Brake_Exec();
            break;

        case CAR_MODE_DONE:
            MODE_Done_Exec();
            break;

        case CAR_MODE_TASK1:
            Task1_Exec();
            break;

        case CAR_MODE_TASK2:
            Task2_Exec();
            break;

        case CAR_MODE_TASK3:
            Task3_Exec();
            break;

        case CAR_MODE_TASK4:
            Task4_Exec();
            if (Task4_IsDone()) {
                s_ctx.mode = CAR_MODE_IDLE;
            }
            break;

        default:
            MODE_Idle_Exec();
            break;
    }
}

CarMode_e Mode_GetMode(void)
{
    return s_ctx.mode;
}

/* ======================== 模式切换 ======================== */

void Mode_SetIdle(void)
{
    s_ctx.mode = CAR_MODE_IDLE;
    MODE_Idle_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_IDLE);
}

void Mode_SetStraight(float base_pwm, float target_yaw)
{
    s_ctx.mode       = CAR_MODE_STRAIGHT;
    s_ctx.base_pwm   = base_pwm;
    s_ctx.target_yaw = target_yaw;
    MODE_Straight_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_STRAIGHT);
}

void Mode_SetTracking(float max_pwm)
{
    s_ctx.mode    = CAR_MODE_TRACKING;
    s_ctx.max_pwm = max_pwm;
    MODE_Tracking_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_TRACKING);
}

void Mode_SetTurning(float target_yaw)
{
    s_ctx.mode       = CAR_MODE_TURNING;
    s_ctx.target_yaw = target_yaw;
    MODE_Turning_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_TURNING);
}

void Mode_SetCurve(float base_pwm, float target_yaw)
{
    s_ctx.mode       = CAR_MODE_CURVE;
    s_ctx.base_pwm   = base_pwm;
    s_ctx.target_yaw = target_yaw;
    MODE_Curve_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_CURVE);
}

void Mode_SetBrake(void)
{
    s_ctx.mode = CAR_MODE_BRAKE;
    MODE_Brake_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_BRAKE);
}

void Mode_SetDone(void)
{
    s_ctx.mode = CAR_MODE_DONE;
    MODE_Done_Init();
    Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_DONE);
}

void Mode_SetTask(uint8_t task_id)
{
    switch (task_id) {
        case 1:
            s_ctx.mode = CAR_MODE_TASK1;
            Task1_Init();
            Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_TASK1);
            Broker_Publish_Int(TOPIC_TASK_ID, 1);
            break;
        case 2:
            s_ctx.mode = CAR_MODE_TASK2;
            Task2_Init();
            Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_TASK2);
            Broker_Publish_Int(TOPIC_TASK_ID, 2);
            break;
        case 3:
            s_ctx.mode = CAR_MODE_TASK3;
            Task3_Init();
            Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_TASK3);
            Broker_Publish_Int(TOPIC_TASK_ID, 3);
            break;
        case 4:
            s_ctx.mode = CAR_MODE_TASK4;
            Task4_Init();
            Broker_Publish_Int(TOPIC_CAR_MODE, (int32_t)CAR_MODE_TASK4);
            Broker_Publish_Int(TOPIC_TASK_ID, 4);
            break;
        default:
            Mode_SetIdle();
            Broker_Publish_Int(TOPIC_TASK_ID, 0);
            break;
    }
}

/* ======================== 查询接口 ======================== */

uint8_t Mode_IsTurningDone(void)
{
    if (s_ctx.mode != CAR_MODE_TURNING) return 0;
    return MODE_Turning_IsDone();
}

uint8_t Mode_IsBrakeDone(void)
{
    if (s_ctx.mode != CAR_MODE_BRAKE) return 0;
    return MODE_Brake_IsDone();
}
