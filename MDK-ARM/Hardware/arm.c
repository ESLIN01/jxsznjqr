/* arm.c —— 机械臂 4 路舵机（TIM1/TIM2 寄存器级 PWM） */
#include "arm.h"

/* 舵机通道对应关系：
 *   舵机1 = TIM1->CCR1 (PA8)   舵机2 = TIM1->CCR4 (PA11)
 *   舵机3 = TIM2->CCR1 (PA0)   舵机4 = TIM2->CCR2 (PA1, 夹爪)          */

/* 夹爪(舵机4)夹紧值：夹紧动作与"末尾姿态"共用同一个值，
 * 这样末尾动作渐变时夹爪不会从夹紧值慢慢松开（否则可能掉块） */
#define ARM_GRIP_US   1000
#define ARM_GRIP_OPEN_US 500   /* 松爪(张开)值：与上电初始/抓取准备姿态用同一个张开值 */

/* 姿态表：舵机1 / 舵机2 / 舵机3 / 舵机4 */
static const int32_t arm_pose_grab[4] = {1421, 1700, 1350, 500};        /* 抓取准备（到夹取点时摆） */
static const int32_t arm_pose_end[4]  = {1421,  600,  850, ARM_GRIP_US};/* 末尾姿态 */

/* 渐变状态（抓取姿态、夹爪夹紧、末尾动作 共用一套） */
static int32_t  arm_ramp_from[4];      /* 渐变起点（启动那一刻的实际脉宽） */
static int32_t  arm_ramp_to[4];        /* 渐变目标 */
static uint32_t arm_ramp_ms;           /* 本次渐变总时长(ms) */
static uint32_t arm_ramp_tick;         /* 渐变开始时刻 */
static uint8_t  arm_ramp_active = 0U;  /* 0=没在渐变 */

/* 抓取序列步骤：0=没开始  1=正在摆抓取姿态  2=正在夹紧  3=整个抓取完成 */
static uint8_t  arm_grab_step = 0U;

/* ================= 舵机 PWM 初始化 =================
 * 舵机1: PA8  = TIM1_CH1    舵机2: PA11 = TIM1_CH4
 * 舵机3: PA0  = TIM2_CH1    舵机4: PA1  = TIM2_CH2
 * 20ms 周期(50Hz)、1MHz 计数 → CCR 数值 = 脉宽 us
 */
void ARM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PA0/PA1(TIM2) + PA8/PA11(TIM1) → 复用推挽输出 */
    GPIO_InitStruct.Pin   = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ---- TIM2（普通定时器）：CH1=PA0 舵机3, CH2=PA1 舵机4(夹爪) ---- */
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC   = 71;                    /* 72MHz/72 = 1MHz → 1us/计数 */
    TIM2->ARR   = 19999;                 /* 周期 20ms */
    TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1   /* CH1 = PWM1 */
                | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;   /* CH2 = PWM1 */
    TIM2->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM2->CCR1  = 1500;                  /* 默认中立 */
    TIM2->CCR2  = 1500;
    TIM2->EGR   = TIM_EGR_UG;
    TIM2->CR1   = TIM_CR1_CEN;           /* 启动 */

    /* ---- TIM1（高级定时器）：CH1=PA8 舵机1, CH4=PA11 舵机2 ---- */
    __HAL_RCC_TIM1_CLK_ENABLE();
    TIM1->PSC   = 71;
    TIM1->ARR   = 19999;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;   /* CH1 = PWM1 */
    TIM1->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;   /* CH4 = PWM1 */
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1->CCR1  = 1500;                  /* 默认中立 */
    TIM1->CCR4  = 1500;
    TIM1->BDTR  = TIM_BDTR_MOE;          /* 主输出使能（高级定时器必须） */
    TIM1->EGR   = TIM_EGR_UG;
    TIM1->CR1   = TIM_CR1_CEN;           /* 启动 */
}

/* ================= 上电初始动作（车还没走时先摆好） =================
 * 1421 / 500 / 1750 / 500 */
void ARM_PoseInit(void)
{
    TIM1->CCR1 = 1421;   /* 舵机1 */
    TIM1->CCR4 = 500;    /* 舵机2 */
    TIM2->CCR1 = 1750;   /* 舵机3 */
    TIM2->CCR2 = 500;    /* 舵机4(夹爪)：上电张开 */
}

/* 开始一次渐变：从"当前实际脉宽" → to[4]，用时 ms
 * （from/to 最大差约 1300，elapsed<3000 → 乘法最大 ~4e6，int32 不会溢出） */
static void ARM_RampBegin(const int32_t to[4], uint32_t ms)
{
    arm_ramp_from[0] = (int32_t)TIM1->CCR1;
    arm_ramp_from[1] = (int32_t)TIM1->CCR4;
    arm_ramp_from[2] = (int32_t)TIM2->CCR1;
    arm_ramp_from[3] = (int32_t)TIM2->CCR2;
    arm_ramp_to[0] = to[0];
    arm_ramp_to[1] = to[1];
    arm_ramp_to[2] = to[2];
    arm_ramp_to[3] = to[3];
    arm_ramp_ms = (ms == 0U) ? 1U : ms;
    arm_ramp_tick = HAL_GetTick();
    arm_ramp_active = 1U;
}

/* ================= 只做"夹爪夹紧"：舵机4 在 ARM_GRAB_CLOSE_MS 内合到 ARM_GRIP_US(1000) =================
 * 前三个舵机保持当前值（目标 = 当前值） */
void ARM_GrabClose(void)
{
    int32_t to[4];

    to[0] = (int32_t)TIM1->CCR1;
    to[1] = (int32_t)TIM1->CCR4;
    to[2] = (int32_t)TIM2->CCR1;
    to[3] = ARM_GRIP_US;
    ARM_RampBegin(to, ARM_GRAB_CLOSE_MS);
}

/* ================= 松爪（放块）：舵机4 在 ARM_GRIP_OPEN_MS 内张到 ARM_GRIP_OPEN_US(500) =================
 * 只动舵机4，前三个舵机保持当前值（目标 = 当前值）；
 * 由 main.c 按"第36秒完成"反推开始时刻后调用 */
void ARM_ReleaseStart(void)
{
    int32_t to[4];

    to[0] = (int32_t)TIM1->CCR1;
    to[1] = (int32_t)TIM1->CCR4;
    to[2] = (int32_t)TIM2->CCR1;
    to[3] = ARM_GRIP_OPEN_US;
    ARM_RampBegin(to, ARM_GRIP_OPEN_MS);
}

/* ================= 到夹取点：① 慢慢摆到"抓取准备"姿态 →（ARM_Task 里接着）② 夹紧 ================= */
void ARM_GrabStart(void)
{
    arm_grab_step = 1U;
    ARM_RampBegin(arm_pose_grab, ARM_GRAB_POSE_RAMP_MS);
}

/* ================= 抓取序列是否还在进行 =================
 * 1 = 正在摆抓取姿态(1) 或 正在夹紧(2)；0 = 还没开始(0) 或 已经做完(3)
 * 没开始也算"不忙"，所以机械臂这一轮没被触发时车也不会被卡住 */
uint8_t ARM_GrabBusy(void)
{
    return ((arm_grab_step == 1U) || (arm_grab_step == 2U)) ? 1U : 0U;
}

/* ================= 末尾姿态（立即到位，一般由 ARM_Task 渐变完成） ================= */
void ARM_PoseEnd(void)
{
    TIM1->CCR1 = (uint32_t)arm_pose_end[0];   /* 舵机1 */
    TIM1->CCR4 = (uint32_t)arm_pose_end[1];   /* 舵机2 */
    TIM2->CCR1 = (uint32_t)arm_pose_end[2];   /* 舵机3 */
    TIM2->CCR2 = (uint32_t)arm_pose_end[3];   /* 舵机4(夹爪)：保持夹紧值不松 */
}

/* 线性插值：pos = from + (to - from) * elapsed / total */
static uint32_t ARM_RampValue(int32_t from, int32_t to, uint32_t elapsed, uint32_t total)
{
    int32_t value;
    value = from + ((to - from) * (int32_t)elapsed) / (int32_t)total;
    if (value < 0) value = 0;
    return (uint32_t)value;
}

/* ================= 开始"慢慢摆末尾动作" =================
 * 以调用这一刻各舵机的实际脉宽为起点，之后由 ARM_Task() 按时间线性渐变到末尾姿态 */
void ARM_EndRampStart(void)
{
    ARM_RampBegin(arm_pose_end, ARM_END_RAMP_MS);
}

/* ================= 主循环每拍调用：推进渐变 / 推进抓取序列 ================= */
void ARM_Task(void)
{
    uint32_t elapsed;

    /* 抓取序列：姿态渐变一结束，紧接着夹紧舵机4 */
    if ((arm_grab_step == 1U) && (arm_ramp_active == 0U))
    {
        ARM_GrabClose();                 /* 进入第 2 步：夹爪夹紧 */
        arm_grab_step = 2U;
    }
    else if ((arm_grab_step == 2U) && (arm_ramp_active == 0U))
    {
        arm_grab_step = 3U;              /* 夹紧也完成，整个抓取结束 */
    }

    if (arm_ramp_active == 0U)
    {
        return;                          /* 没在渐变：什么都不做 */
    }

    elapsed = (uint32_t)(HAL_GetTick() - arm_ramp_tick);
    if (elapsed >= arm_ramp_ms)
    {
        TIM1->CCR1 = (uint32_t)arm_ramp_to[0];   /* 结束：精确落到目标 */
        TIM1->CCR4 = (uint32_t)arm_ramp_to[1];
        TIM2->CCR1 = (uint32_t)arm_ramp_to[2];
        TIM2->CCR2 = (uint32_t)arm_ramp_to[3];
        arm_ramp_active = 0U;
        return;
    }

    TIM1->CCR1 = ARM_RampValue(arm_ramp_from[0], arm_ramp_to[0], elapsed, arm_ramp_ms);
    TIM1->CCR4 = ARM_RampValue(arm_ramp_from[1], arm_ramp_to[1], elapsed, arm_ramp_ms);
    TIM2->CCR1 = ARM_RampValue(arm_ramp_from[2], arm_ramp_to[2], elapsed, arm_ramp_ms);
    TIM2->CCR2 = ARM_RampValue(arm_ramp_from[3], arm_ramp_to[3], elapsed, arm_ramp_ms);
}
