/**
 * @file  MODE_TASK3.c
 * @brief 任务3 完整版（融合红外 + IMU + 里程计）：
 *   流程：
 *   1. A→C 快速起步→线性减速斜插C点，碰线微调身位对准黑线
 *   2. C点向内扫线（未碰到时）→ 直接进循迹（记忆救车兜底）
 *   3. CB循迹半圆（记忆救车）
 *   4. B丢线 → 弧线脱离+直走至 180° → 转到 ~131° → BD斜切直走
 *   5. D点：斜切途中碰线直接入循迹 / 没碰到 → 转 180° → 慢走找线
 *   6. DA循迹半圆（记忆救车）
 *   7. A点对齐 0° → 停车
 *
 *   适配 Broker + 轮速闭环架构
 */
#include "MODE_TASK3.h"
#include "Application/broker/broker.h"
#include "Device/track/track.h"
#include "Mode/MODE_BRAKE.h"
#include "Mode/MODE_CURVE.h"
#include "Mode/MODE_STRAIGHT.h"
#include "Mode/MODE_TRACKING.h"
#include "Mode/MODE_TURNING.h"
#include "config.h"
#include <math.h>

/* ======================== 步骤枚举 ======================== */
typedef enum {
  /* A→C 线性减速斜插段 */
  T3_AC_APPROACH = 0, /* 快速→线性减速斜插C点 */

  /* C 点向内扫线 */
  T3_C_SWEEP_IN, /* 向内扫转找线 */

  /* CB 循迹半圆 */
  T3_CB_TRACK, /* 循迹半圆 */

  /* B 点脱离 */
  T3_B_DEPART, /* 弧线+直走脱离到 180° 方向 */

  /* B→D 斜切段 */
  T3_BD_TURN,     /* 原地转到 ~131° */
  T3_BD_STRAIGHT, /* 里程计控制直走找D线 */
  T3_BD_BRAKE,    /* 斜切途中碰线制动 */

  /* D 点扫头找线 */
  T3_D_TURN_180,     /* 没碰到线，转 180° */
  T3_D_SLOW_FORWARD, /* 慢走前进找线 */
  T3_D_BRAKE,        /* 找到线后制动沉降 */

  /* DA 循迹半圆 */
  T3_DA_TRACK, /* 循迹半圆 */

  /* A 点收尾 */
  T3_A_BRAKE,     /* A点刹车 */
  T3_A_TURN_0,    /* 原地对齐 0° */
  T3_FINAL_BRAKE, /* 最终停车 */
  T3_DONE
} Task3Step_e;

/* ======================== 内部状态 ======================== */
static Task3Step_e s_step = T3_AC_APPROACH;
static uint8_t s_done = 0;
static float s_base_yaw = 0.0f;   /* 初始航向 */
static float s_target_yaw = 0.0f; /* 当前目标航向 */
static uint16_t s_tick = 0;
static uint16_t s_track_valid = 0;
static uint16_t s_track_lost = 0;
static uint8_t s_line_confirm = 0;
static uint8_t s_stable_cnt = 0;      /* 速度判停计数 */
static float s_dist_snap = 0.0f;      /* 里程快照 */
static float s_last_valid_err = 0.0f; /* 记忆救车：最后有效循迹误差 */
static uint8_t s_lap_count = 0;       /* 已完成圈数 */
static float s_align_tol = 1.0f;      /* A点对齐容忍度（度） */

#define LINE_FIND_COUNT 2    /* 寻线碰线确认（灵敏：快速检测C/D点线）*/
#define LINE_LOST_CONFIRM 5  /* 循迹中碰线确认（抗噪：防误触发状态跳转）*/

/* ---- 工具函数 ---- */
static float _normalize(float a) {
  while (a > 180.0f)
    a -= 360.0f;
  while (a < -180.0f)
    a += 360.0f;
  return a;
}

static void _reset(void) {
  s_track_valid = 0;
  s_track_lost = 0;
  s_line_confirm = 0;
  s_stable_cnt = 0;
  s_last_valid_err = 0.0f;
}

/* 获取左右轮平均里程 (mm) - 代数和，仅计算车体中心真实位移 */
static float _get_mileage(void) {
  float dl = Broker_Subscribe_Float(TOPIC_DISTANCE_LEFT);
  float dr = Broker_Subscribe_Float(TOPIC_DISTANCE_RIGHT);
  return (dl + dr) * 0.5f;
}

/* 重置里程快照（在段落起始调用） */
static void _snap_dist(void) { s_dist_snap = _get_mileage(); }

/* 获取自快照以来行驶的距离 */
static float _get_dist(void) { return fabsf(_get_mileage() - s_dist_snap); }

/* 速度判停：返回 1 = 已停稳 */
static uint8_t _is_stopped(void) {
  float sl = Broker_Subscribe_Float(TOPIC_SPEED_LEFT);
  float sr = Broker_Subscribe_Float(TOPIC_SPEED_RIGHT);
  if (fabsf(sl) < BRAKE_SPEED_THRESHOLD && fabsf(sr) < BRAKE_SPEED_THRESHOLD) {
    s_stable_cnt++;
  } else {
    s_stable_cnt = 0;
  }
  return (s_stable_cnt >= BRAKE_STABLE_COUNT) ? 1 : 0;
}

/* ======================== 公共接口 ======================== */

void Task3_Init(void) {
  s_step = T3_AC_APPROACH;
  s_done = 0;
  s_tick = 0;
  s_lap_count = 0;
  _reset();

  s_base_yaw = Broker_Subscribe_Float(TOPIC_JY61P_YAW);
  s_target_yaw = _normalize(s_base_yaw + 39.66f);

  _snap_dist();
  MODE_Curve_Init();
}

void Task3_Exec(void) {
  if (s_done)
    return;

  float yaw = Broker_Subscribe_Float(TOPIC_JY61P_YAW);
  float track_err = Broker_Subscribe_Float(TOPIC_TRACK_ERROR);

  switch (s_step) {
  /* ================================================================
   *  A → C 线性减速斜插段
   * ================================================================ */

  /* ===== 1. 快速起步→线性减速斜插C点 ===== */
  case T3_AC_APPROACH: {
    float dist = _get_dist();
    float yaw_err = _normalize(s_target_yaw - yaw);
    s_tick++;

    /* 速度分段：先全速，然后线性减速 */
    float spd;
    if (dist < TASK3_AC_FULL_SPEED_DIST) {
      spd = TASK_STRAIGHT_FAST;
    } else {
      float decel_progress =
          (dist - TASK3_AC_FULL_SPEED_DIST) / TASK3_AC_RAMP_DIST;
      if (decel_progress > 1.0f)
        decel_progress = 1.0f;
      spd = TASK_STRAIGHT_FAST -
            (TASK_STRAIGHT_FAST - TASK3_AC_SLOW_SPEED) * decel_progress;
    }

    /* 起步柔和加速：前5个tick从30%线性爬到100% */
    if (s_tick < 5) {
      float ramp = 0.3f + 0.7f * ((float)s_tick / 5.0f);
      spd *= ramp;
    }

    /* 航向控制：大角度用弧线模式，小角度用直线模式 */
    if (fabsf(yaw_err) > 5.0f) {
      MODE_Curve_Exec(spd, s_target_yaw, yaw);
    } else {
      MODE_Straight_Exec(spd, s_target_yaw, yaw);
    }

    /* 碰线检测 (跑 80mm 后开始，防起步误触) */
    if (dist > 400.0f && track_err != TRACK_LOST_LINE) {
      s_line_confirm++;
    } else {
      s_line_confirm = 0;
    }

    if (s_line_confirm >= LINE_FIND_COUNT) {
      /* 碰线 → 直接进循迹（速度400与TASK_TRACK_HIGH无缝衔接） */
      _reset();
      MODE_Tracking_Init();
      s_step = T3_CB_TRACK;
      break;
    }

    /* 距离到位没碰线 → 直接进循迹（记忆救车兜底） */
    if (dist >= TASK3_STRAIGHT_MAX_DIST) {
      _reset();
      MODE_Tracking_Init();
      s_step = T3_CB_TRACK;
    }
    break;
  }

  /* ================================================================
   *  C 点向内扫线
   * ================================================================ */

  /* ===== 2. 向内扫转找线 ===== */
  case T3_C_SWEEP_IN: {
    /* 向内扫线：朝圆弧内侧转动寻找黑线 */
    float sweep_yaw = _normalize(s_base_yaw + SWEEP_ANGLE_C);
    float sweep_err = _normalize(sweep_yaw - yaw);

    /* 扫转过程中检测红外 */
    if (track_err != TRACK_LOST_LINE) {
      /* 扫到线了！→ 直接进循迹 */
      Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
      Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
      _reset();
      MODE_Tracking_Init();
      s_step = T3_CB_TRACK;
      break;
    }

    s_tick++;

    /* 扫转到位 or 超时 → 直接进循迹（记忆救车兜底） */
    if (fabsf(sweep_err) < 3.0f || s_tick > SWEEP_TIMEOUT) {
      Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
      Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
      _reset();
      MODE_Tracking_Init();
      s_step = T3_CB_TRACK;
    } else {
      MODE_Turning_Exec(sweep_yaw, yaw);
    }
    break;
  }

    /* ================================================================
     *  CB 循迹半圆 (记忆救车)
     * ================================================================ */

  case T3_CB_TRACK: {
    if (track_err != TRACK_LOST_LINE) {
      s_track_lost = 0;
      s_last_valid_err = track_err;
      if (s_track_valid < 60000)
        s_track_valid++;
    } else {
      s_track_lost++;
    }

    /* B点丢线判定 */
    if ((s_track_valid > 400 && s_track_lost > 40) || s_track_lost > 300) {
      /* 弧线脱离 */
      MODE_Curve_Init();
      _snap_dist();
      s_tick = 0;
      s_target_yaw = _normalize(s_base_yaw + 180.0f);
      s_step = T3_B_DEPART;
      break;
    }

    /* 记忆救车：根据上一次稳定循迹方向恢复 */
    if (track_err == TRACK_LOST_LINE) {
      float recovery = (s_last_valid_err > 0) ? 40.0f : -40.0f;
      MODE_Tracking_Exec(TASK_TRACK_LOW, recovery);
    } else {
      MODE_Tracking_Exec(TASK_TRACK_HIGH, track_err);
    }
    break;
  }

    /* ================================================================
     *  B 点脱离 (弧线+直走到 180° 方向)
     * ================================================================ */

  case T3_B_DEPART: {
    float yaw_err = _normalize(s_target_yaw - yaw);
    float dist = _get_dist();
    s_tick++;

    /* 行进间碰线 → 直接入 DA 循迹 */
    if (s_tick > 5 && track_err != TRACK_LOST_LINE) {
      s_line_confirm++;
    } else {
      s_line_confirm = 0;
    }
    if (s_line_confirm >= LINE_LOST_CONFIRM) {
      _reset();
      MODE_Tracking_Init();
      s_step = T3_DA_TRACK;
      break;
    }

    /* 航向差大 → 弧线，差小 → 直线 */
    if (fabsf(yaw_err) > 5.0f) {
      MODE_Curve_Exec(TASK_STRAIGHT_MID, s_target_yaw, yaw);
    } else {
      MODE_Straight_Exec(TASK_STRAIGHT_FAST, s_target_yaw, yaw);
    }

    /* 脱离距离到位 → 转向 BD 斜切角 */
    if (dist >= 150.0f) {
      s_target_yaw = _normalize(s_base_yaw + 140.0f);
      MODE_Turning_Init();
      s_tick = 0;
      s_step = T3_BD_TURN;
    }
    break;
  }

  /* ================================================================
   *  B → D 斜切段
   * ================================================================ */

  /* ===== 5. 原地转到 ~131° ===== */
  case T3_BD_TURN: {
    MODE_Turning_Exec(s_target_yaw, yaw);

    if (MODE_Turning_IsDone()) {
      MODE_Straight_Init();
      _snap_dist();
      s_tick = 0;
      s_line_confirm = 0;
      s_step = T3_BD_STRAIGHT;
    }
    break;
  }

  /* ===== 5.5 BD 直走 (里程计+碰线检测) ===== */
  case T3_BD_STRAIGHT: {
    float dist = _get_dist();
    s_tick++;

    /* 里程分段变速 */
    float spd;
    if (dist < TASK3_STRAIGHT_DECEL_DIST) {
      spd = TASK_STRAIGHT_FAST;
    } else {
      spd = TASK_STRAIGHT_MID;
    }
    MODE_Straight_Exec(spd, s_target_yaw, yaw);

    /* 途中碰线 → 制动后直接入循迹 */
    if (dist > 100.0f && track_err != TRACK_LOST_LINE) {
      s_line_confirm++;
      s_last_valid_err = track_err; /* 保存误差方向供DA恢复用 */
    } else {
      s_line_confirm = 0;
    }
    if (s_line_confirm >= LINE_LOST_CONFIRM) {
      Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
      Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
      s_tick = 0;
      s_stable_cnt = 0;
      s_step = T3_BD_BRAKE;
      break;
    }

    /* 兜底：没碰到线，超距后 → D 点扫线 */
    if (dist >= TASK3_STRAIGHT_MAX_DIST) {
      Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
      Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
      s_tick = 0;
      s_stable_cnt = 0;
      s_step = T3_D_TURN_180;
    }
    break;
  }

  /* ===== 5.6 BD斜切途中碰线制动 → 入循迹 ===== */
  case T3_BD_BRAKE: {
    Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
    Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
    s_tick++;
    if (_is_stopped() || s_tick > SWEEP_BRAKE_CNT) {
      _reset();
      s_last_valid_err = 3.0f; /* 两个半圆都是顺时针(yaw递减)，恢复均向右 */
      MODE_Tracking_Init();
      _snap_dist();
      s_step = T3_DA_TRACK;
    }
    break;
  }

  /* ================================================================
   *  D 点扫头找线
   * ================================================================ */

  /* ===== 6. D 点转 180° 对齐切线方向 ===== */
  case T3_D_TURN_180: {
    s_target_yaw = _normalize(s_base_yaw + 180.0f);

    /* 首次进入初始化转向 */
    if (s_tick == 0) {
      MODE_Turning_Init();
    }
    s_tick++;

    MODE_Turning_Exec(s_target_yaw, yaw);

    if (MODE_Turning_IsDone()) {
      MODE_Straight_Init();
      _snap_dist();
      s_tick = 0;
      s_step = T3_D_SLOW_FORWARD;
    }
    break;
  }

  /* ===== 6.5 D 点慢速前进找线 ===== */
  case T3_D_SLOW_FORWARD: {
    float dist = _get_dist();

    if (track_err != TRACK_LOST_LINE || dist >= 300.0f) {
      /* 碰线 or 超距 → 制动沉降 */
      Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
      Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
      s_tick = 0;
      s_stable_cnt = 0;
      s_step = T3_D_BRAKE;
    } else {
      MODE_Straight_Exec(TASK_STRAIGHT_SLOW, s_target_yaw, yaw);
    }
    break;
  }

  /* ===== 6.6 D 点制动沉降后入循迹 ===== */
  case T3_D_BRAKE: {
    Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
    Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
    s_tick++;
    if (_is_stopped() || s_tick > SWEEP_BRAKE_CNT) {
      _reset();
      MODE_Tracking_Init();
      _snap_dist();
      s_step = T3_DA_TRACK;
    }
    break;
  }

    /* ================================================================
     *  DA 循迹半圆 (记忆救车)
     * ================================================================ */

  case T3_DA_TRACK: {
    float dist = _get_dist();

    if (track_err != TRACK_LOST_LINE) {
      s_track_lost = 0;
      /* 稳定循迹20+tick后才更新方向，防瞬间误读 */
      if (s_track_valid > 20) {
        s_last_valid_err = track_err;
      }
      if (s_track_valid < 60000)
        s_track_valid++;
    } else {
      s_track_lost++;
    }

    /* A点丢线判定：走完大部分半圆(~1100mm/1257mm) + 丢线 */
    if ((dist > 1100.0f && s_track_lost > 0) || s_track_lost > 400) {
      Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
      Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
      s_tick = 0;
      s_stable_cnt = 0;
      s_step = T3_A_BRAKE;
      break;
    }

    /* 记忆救车：根据上一次稳定循迹方向恢复 */
    if (track_err == TRACK_LOST_LINE) {
      float recovery = (s_last_valid_err > 0) ? 40.0f : -40.0f;
      MODE_Tracking_Exec(TASK_TRACK_LOW, recovery);
    } else {
      /* 接近A点时自动减速，减小冲过惯性 */
      float spd = (dist > 1000.0f) ? TASK_TRACK_MID : TASK_TRACK_HIGH;
      MODE_Tracking_Exec(spd, track_err);
    }
    break;
  }

  /* ================================================================
   *  A 点收尾
   * ================================================================ */

  /* ===== 8. A 点速度判停 ===== */
  case T3_A_BRAKE: {
    Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
    Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
    s_tick++;
    if (_is_stopped() || s_tick > 50) {
      s_target_yaw = s_base_yaw;
      MODE_Turning_Init();
      s_tick = 0;
      s_step = T3_A_TURN_0;
    }
    break;
  }

  /* ===== 9. A点精对齐0°（快粗转→慢精修→等待确认）===== */
  case T3_A_TURN_0: {
    float yaw_err = _normalize(s_target_yaw - yaw);
    float abs_err = fabsf(yaw_err);

    if (abs_err > 5.0f) {
      /* 阶段1：大误差 → MODE_Turning PID粗转 */
      MODE_Turning_Exec(s_target_yaw, yaw);
      s_tick = 0;
    } else if (abs_err > 2.0f) {
      /* 阶段2：小误差 → 60mm/s 慢速精修（死区2°防振荡） */
      float turn_spd = 60.0f;
      if (yaw_err > 0) {
        Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, -turn_spd);
        Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, turn_spd);
      } else {
        Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, turn_spd);
        Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, -turn_spd);
      }
      s_tick = 0;
    } else if (s_tick <= 10) {
      /* 阶段3：到位，停车等 50ms 惯性消退 */
      Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
      Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
      s_tick++;
    } else {
      /* 阶段4：确认残差 */
      if (abs_err > s_align_tol) {
        MODE_Turning_Init();
        s_tick = 0;
      } else {
        s_tick = 0;
        s_stable_cnt = 0;
        s_step = T3_FINAL_BRAKE;
      }
    }
    break;
  }

  /* ===== 10. 最终停车 ===== */
  case T3_FINAL_BRAKE: {
    Broker_Publish_Float(TOPIC_TARGET_SPEED_LEFT, 0.0f);
    Broker_Publish_Float(TOPIC_TARGET_SPEED_RIGHT, 0.0f);
    s_tick++;
    if (_is_stopped() || s_tick > 50) {
      s_step = T3_DONE;
      s_done = 1;
    }
    break;
  }

  default:
    break;
  }
}

uint8_t Task3_IsDone(void) { return s_done; }

void Task3_SetBaseYaw(float yaw) {
    s_base_yaw   = yaw;
    s_target_yaw = _normalize(s_base_yaw + 39.66f);
}

void Task3_SetSkipAlign(uint8_t skip) {
    (void)skip; /* 废弃，用 Task3_SetAlignTol 替代 */
}

void Task3_SetAlignTol(float tol) {
    s_align_tol = tol;
}

uint8_t Task3_IsAtA(void) {
    return (s_step == T3_A_BRAKE) ? 1 : 0;
}
