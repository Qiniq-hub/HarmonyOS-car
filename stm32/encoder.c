#include "encoder.h"

// 自动重装载值（根据需求调整，此处设为65535，即16位定时器最大值）
#define ENCODER_TIM_PERIOD 65535


/**
 * @brief  初始化 TIM2 为编码器接口模式（左电机，PA0/PA1）
 */
void Encoder_Init_TIM2(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能时钟：TIM2（APB1）、GPIOA（APB2）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 配置 GPIOA 引脚（PA0、PA1）为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 初始化 TIM2 时基
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;              // 预分频器（不分频）
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  // 自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频（不分频）
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 配置编码器接口模式（模式3：双通道捕获）
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    // 输入捕获滤波（抗干扰）
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    // 清除更新标志位 + 使能更新中断 + 启动定时器
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_SetCounter(TIM2, 0);   // 计数器清零
    TIM_Cmd(TIM2, ENABLE);     // 启动 TIM2
}


/**
 * @brief  初始化 TIM3 为编码器接口模式（右电机，PA6/PA7）
 */
void Encoder_Init_TIM3(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能时钟：TIM3（APB1）、GPIOA（APB2）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 配置 GPIOA 引脚（PA6、PA7）为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 初始化 TIM3 时基
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;              // 预分频器（不分频）
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  // 自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频（不分频）
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 配置编码器接口模式（模式3：双通道捕获）
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    // 输入捕获滤波（抗干扰）
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    // 清除更新标志位 + 使能更新中断 + 启动定时器
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_SetCounter(TIM3, 0);   // 计数器清零
    TIM_Cmd(TIM3, ENABLE);     // 启动 TIM3
}


/**
 * @brief  读取指定定时器的编码器计数值（速度）
 * @param  TIMX: 定时器编号（2=左电机，3=右电机）
 * @return 速度值（CNT 寄存器的差值）
 */
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM = 0;

    switch (TIMX)
    {
        case 2:  // 左电机（TIM2）
            TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除中断标志
            Encoder_TIM = (short)TIM2->CNT;             // 读取 CNT（强制转换处理溢出）
            TIM_SetCounter(TIM2, 0);                    // 计数器清零
            break;

        case 3:  // 右电机（TIM3）
            TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // 清除中断标志
            Encoder_TIM = (short)TIM3->CNT;             // 读取 CNT
            TIM_SetCounter(TIM3, 0);                    // 计数器清零
            break;

        default:
            Encoder_TIM = 0;
            break;
    }

    return Encoder_TIM;
}
