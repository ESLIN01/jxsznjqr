/* arm.h */
#ifndef __ARM_H
#define __ARM_H

#include "main.h"

/*
 * 机械臂 4 路舵机（寄存器级 PWM）
 *   舵机1: PA8  = TIM1_CH1 (TIM1->CCR1)   1421（全程基本不动）
 *   舵机2: PA11 = TIM1_CH4 (TIM1->CCR4)   上电 500  → 抓取准备 1700 → 末尾 600
 *   舵机3: PA0  = TIM2_CH1 (TIM2->CCR1)   上电 1750 → 抓取准备 1350 → 末尾 850
 *   舵机4: PA1  = TIM2_CH2 (TIM2->CCR2)   夹爪：上电 500（张开）→ 夹紧 1000 → 第36秒 松开回 500
 * 20ms 周期(50Hz)、1MHz 计数 → CCR 数值 = 脉宽 us（500~2500）
 * 这几个脚在本工程里没有被其它外设占用（TIM3=PC6~PC9 电机、SPI2=PB13~PB15 IMU、
 * 红外=PC11~PC14/PB3~PB5/PB12、PC10=推挽输出），所以直接寄存器初始化即可。
 * 所有"慢慢动"的动作都在主循环 ARM_Task() 里做时间驱动渐变，不占 20ms 中断。
 */

#define ARM_END_RAMP_MS         3000U   /* 末尾动作渐变总时长(ms) */
#define ARM_GRAB_POSE_RAMP_MS   1500U   /* 到夹取点：摆"抓取准备"姿态的渐变时长(ms) */
#define ARM_GRAB_CLOSE_MS        300U   /* 姿态摆好后夹紧舵机4 的时长(ms)：设 0 = 瞬间夹紧 */
#define ARM_GRIP_OPEN_MS         500U   /* 松爪(张开)用时(ms)：main.c 用它反推开始时刻，保证"第36秒完成" */

void ARM_Init(void);         /* 舵机 PWM 初始化（上电先给中位 1500） */
void ARM_PoseInit(void);     /* 上电初始动作：1421 / 500 / 1750 / 500（夹爪张开） */

/* 到夹取点调用：① ARM_GRAB_POSE_RAMP_MS 内摆到抓取准备姿态 1421/1700/1350/500
 *               ② 摆好后自动接着把舵机4 夹紧到 1000（用时 ARM_GRAB_CLOSE_MS） */
void ARM_GrabStart(void);
void ARM_GrabClose(void);    /* 只做"夹爪夹紧"这一步（前三个舵机保持不动） */

/* 抓取序列是否还在进行：1 = 正在摆抓取姿态或正在夹紧；0 = 还没开始 或 已经做完
 * （stage.c 的夹取点收尾用它保证"抓取动作做完再走"） */
uint8_t ARM_GrabBusy(void);

void ARM_ReleaseStart(void); /* 松爪（放块）：只动舵机4，ARM_GRIP_OPEN_MS 内张到 500（前三个不动） */

void ARM_PoseEnd(void);      /* 末尾姿态：1421 / 600 / 850 / 1000（立即到位；一般不直接调） */
void ARM_EndRampStart(void); /* 以当前姿态为起点，ARM_END_RAMP_MS 内慢慢摆到末尾姿态 */
void ARM_Task(void);         /* 主循环每拍调用：推进渐变与抓取序列；没事情时什么都不做 */

#endif
