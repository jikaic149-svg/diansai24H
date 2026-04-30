/**
 * @file  MODE_CURVE.c
 * @brief 弧线行进模式：前进 + IMU 航向 PID 渐进式转向
 *        与 STRAIGHT 的区别：使用独立 PID 参数，无积分项，D 值略高防超调
 */
#include "MODE_CURVE.h"
#include "Application/broker/broker.h"
#include "Application/algorithm/pid_controller.h"
#include "config.h"

static PIDInstance s_curve_pid;

void MODE_Curve_Init(void)
{
    PID_Init_Config_s cfg = {
        .Kp            = PID_CURVE_KP,
        .Ki            = PID_CURVE_KI,
        .Kd            = PID_CURVE_KD,
        .MaxOut        = MODE_CURVE_MAX_CORRECTION,
        .IntegralLimit = 300.0f,
        .DeadBand      = 0.0f,
        .Improve       = PID_Integral_Limit
    };
    PIDInit(&s_curve_pid, &cfg);
}

void MODE_Curve_Exec(float base_pwm, float target_yaw, float current_yaw)
{
    float yaw_error = target_yaw - current_yaw;
    if (yaw_error >  180.0f) yaw_error -= 360.0f;
    if (yaw_error < -180.0f) yaw_error += 360.0f;

    float turn_out = PIDCalculate(&s_curve_pid, 0.0f, yaw_error, 1.0f);

    float pwm_left  = base_pwm - turn_out;
    float pwm_right = base_pwm + turn_out;

    Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT,  pwm_left);
    Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, pwm_right);
}
