/**
 * @file  MODE_TASK1.h
 * @brief 任务1：直线冲刺，遇到黑线立刻停车（三轮车万向轮适配版）
 */
#ifndef MODE_TASK1_H
#define MODE_TASK1_H

#include <stdint.h>

void    Task1_Init(void);
void    Task1_Exec(void);
uint8_t Task1_IsDone(void);

#endif /* MODE_TASK1_H */
