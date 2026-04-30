#ifndef __CONFIG_H
#define __CONFIG_H

// ==========================================
//  1. 机械与硬件基础参数 (基于 MG310 GMR版)
// ==========================================
#define WHEEL_DIAMETER 48.0f                 // 轮子直径 (单位: mm)
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0f) // 轮子半径 (mm)
#define WHEEL_CIRCUMFERENCE                                                    \
  (3.14159265f * WHEEL_DIAMETER) // 轮子周长 (mm) ≈150.8mm
#define REDUCTION_RATIO 20.0f    // 电机减速比 1:20
#define ENCODER_LINES 13.0f      // 霍尔编码器线数
#define MULTIPLIER 4.0f          // 4 倍频解码
#define TOTAL_RESOLUTION (ENCODER_LINES * MULTIPLIER * REDUCTION_RATIO) // =1040
#define TRACK_WIDTH 128.0f // 左右轮中心距 (单位: mm)
#define WHEEL_BASE 113.0f  // 前后轮轴距 (单位: mm)
// 车辆几何参数 (单位: mm，以车身几何中心为原点 [0,0])（暂时）
#define CAR_LENGTH 210.0f      // 车身总长度 (如包含最前端红外探头到尾部的距离)
#define CENTER_TO_FRONT 100.0f // 几何中心到车头最前端的距离
#define CENTER_TO_REAR 90.0f   // 几何中心到车尾最后端的距离

// ==========================================
// 2. D24A 驱动板输出参数 (防烧车、防死区)
// ==========================================
#define PWM_MAX_LIMIT 999 // PWM 最大值
#define MOTOR_DEADZONE 0  // PID 闭环时由积分项处理死区，设为 0 禁用硬补偿

// ==========================================
// 4. 角度外环 PID 参数 (维持定海神针)
// ==========================================
#define PID_YAW_KP 8.0f  // 直线纠偏 P：快速响应
#define PID_YAW_KI 0.02f // 直线纠偏 I：极小积分消除稳态偏差
#define PID_YAW_KD 3.0f  // 直线纠偏 D：阻尼抑制振荡（配合 dt=1.0）

// Curve 模式独立参数：行进间动态切角，不需要积分，D值略高防超调
#define PID_CURVE_KP 4.5f
#define PID_CURVE_KI 0.0f
#define PID_CURVE_KD 20.0f
#define CURVE_TURN_RATIO 0.8f

// 循迹模式微调 (配合 8531 权重)
#define PID_TRACK_KP 22.0f        // P值
#define PID_TRACK_KI 0.0f         // I值：关闭防过冲
#define PID_TRACK_KD 18.0f        // D值
#define TRACK_MAX_TURN_RATIO 0.9f // 转向限幅占基速 90%

// 自适应变速参数
#define TRACK_CURVE_THRESHOLD 2.5f // 误差 > 2.5 开始减速
#define TRACK_MIN_SPEED_RATIO 0.65f // 急弯最低 65%（减少降速拖慢）
#define TRACK_ERROR_FILTER 0.5f    // 滤波系数（降低加快响应）

// ==========================================
// 6. 任务层速度参数 (轮速闭环, 单位: mm/s)
// ==========================================
#define TASK_STRAIGHT_FAST 650.0f // 直道全速
#define TASK_STRAIGHT_MID 500.0f  // 斜切/弧线段速度
#define TASK_STRAIGHT_SLOW 200.0f // 压线慢速
#define TASK_TRACK_HIGH 450.0f    // 循迹峰值
#define TASK_TRACK_MID 300.0f     // 盲走入线/接近A点减速
#define TASK_TRACK_LOW 220.0f     // 丢线恢复速度
#define TASK3_TOTAL_LAPS 3        // T3 连续行驶圈数

// ==========================================
// 7. 模式层转向与速度参数 (轮速闭环, 单位: mm/s)
// ==========================================
#define STRAIGHT_TURN_RATIO 0.4f            // 直线模式：纠偏占基速的 40%
#define MODE_STRAIGHT_MAX_CORRECTION 350.0f // 直线模式最大航向纠偏 (mm/s)
#define MODE_CURVE_MAX_CORRECTION 450.0f    // 弧线模式最大纠偏 (mm/s)
#define AXIS_MAX_TURN_SPEED 100.0f          // 轴向模式最大转向速度 (mm/s)
#define AXIS_BASE_SPEED 150.0f              // 轴向模式基础前进速度 (mm/s)
#define AXIS_MIN_SPEED 30.0f                // 轴向模式最小前进速度 (mm/s)
#define AXIS_TURN_RATIO 0.7f                // 轴向模式：转向占基速的 70%
#define TURNING_MIN_SPEED 120.0f // 转向模式：克服静摩擦最小轮速
#define TURNING_MAX_SPEED 250.0f // 转向模式：最大轮速
#define PID_TURNING_KD 18.0f      // 转向 D：强阻尼减少过冲振荡
#define TURNING_DECEL_ZONE 15.0f // 转向减速区

// ==========================================
// 8. 模式默认速度参数 (轮速闭环, 单位: mm/s)
// ==========================================
#define MODE_DEFAULT_STRAIGHT_SPEED 300.0f // 直线行驶默认速度 (mm/s)
#define MODE_DEFAULT_TRACK_SPEED 350.0f    // 循迹默认最大速度 (mm/s)
#define MODE_DEFAULT_CURVE_SPEED 180.0f    // 弧线行驶默认速度 (mm/s)

// ==========================================
// 9. 轮速 PID 闭环参数 (目标: mm/s, 输出: PWM)
//    适配范围 450~700 mm/s
// ==========================================
#define PID_WHEEL_KP 3.5f               // 速度环 P：加快初始响应
#define PID_WHEEL_KI 5.0f               // 速度环 I：大幅加速稳态收敛
#define PID_WHEEL_KD 0.005f             // 速度环 D：降低抑制超调
#define PID_WHEEL_MAX_OUT 999.0f        // 输出限幅 = PWM 最大值
#define PID_WHEEL_INTEGRAL_LIMIT 500.0f // 积分限幅 (防饱和)
#define PID_WHEEL_DEADBAND 3.0f         // 死区 (mm/s)
#define PID_WHEEL_FF 0.85f              // 前馈增益 (降低防起步超调)

// ==========================================
// 10. 闭环距离控制参数 (轮速PID已闭环)
// ==========================================
#define BRAKE_SPEED_THRESHOLD 20.0f      // 停稳判定速度阈值 (mm/s)
#define BRAKE_STABLE_COUNT 3             // 连续低速 3 次视为停稳（加快）
#define TASK3_CURVE_MAX_DIST 400.0f      // 弧线段最大行驶距离 (mm)
#define TASK3_STRAIGHT_DECEL_DIST 800.0f // BD段减速距离（推迟）
#define TASK3_STRAIGHT_MAX_DIST 1500.0f  // AC安全超时（1256+244mm裕量）
#define TASK3_AC_FULL_SPEED_DIST 900.0f  // AC全速段（加长）
#define TASK3_AC_RAMP_DIST 300.0f  // AC减速斜坡缩短
#define TASK3_AC_SLOW_SPEED 400.0f // AC末速（降低保证C点碰线）

// ==========================================
// 11. C/D 点扫线参数
// ==========================================
#define SWEEP_ANGLE_C                                                          \
  100.0f // C点向内扫线目标角 (相对base_yaw偏移, 逆时针向圆弧内侧)
#define SWEEP_TIMEOUT 50       // 扫线超时 tick 数
#define SWEEP_SLOW_DIST 200.0f // 扫线慢走最大距离 (mm)
#define SWEEP_BRAKE_CNT 5      // 扫线制动沉降 tick 数

#endif
