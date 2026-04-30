/**
 * @file  MODE_TASK3.h
 * @brief 任务3：复杂多段导航（三轮车万向轮适配版，边走边转）
 */
#ifndef MODE_TASK3_H
#define MODE_TASK3_H

#include <stdint.h>

void    Task3_Init(void);
void    Task3_Exec(void);
uint8_t Task3_IsDone(void);
void    Task3_SetBaseYaw(float yaw);  /* T4用：覆盖base_yaw保持第一圈角度 */
void    Task3_SetSkipAlign(uint8_t skip);
void    Task3_SetAlignTol(float tol);
uint8_t Task3_IsAtA(void);  /* 返回1=T3已到A点刹车阶段 */

#endif /* MODE_TASK3_H */
