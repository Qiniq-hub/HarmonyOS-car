#include "motor.h"
#include "stm32f10x.h" 

/**
  * @brief  绝对值函数 (对应图片5)
  */
u32 myabs(long int a)
{
    u32 temp;
    if(a < 0)
        temp = -a;
    else
        temp = a;
    return temp;
}

/**
  * @brief  电机方向引脚初始化 (对应图片3)
  * @note   配置 PB13, PB14 为推挽输出
  */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能 GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始化为低电平 (停止或默认状态)
    AIN = 0;
    BIN = 0;
}

/**
  * @brief  PWM 初始化 (对应图片4、5)
  * @param  arr: 自动重装值 (决定频率)
  * @param  psc: 预分频系数
  * @note   使用 TIM4，通道1(PB6)和通道2(PB7)
  */
void PWM_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    Motor_Init(); // 先初始化方向引脚
    
    // 1. 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // TIM4时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // GPIOB时钟
    
    // 2. 配置 GPIO (PB6, PB7) 为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 3. 初始化定时器时基
    TIM_TimeBaseStructure.TIM_Period = arr; 
    TIM_TimeBaseStructure.TIM_Prescaler = psc; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
    
    // 4. 初始化 PWM 模式 (通道1 和 通道2)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始占空比为0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
    
    // 初始化通道1 (对应左电机 PWM)
    TIM_OC1Init(TIM4, &TIM_OCInitStructure); 
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); // 使能预装载寄存器
    
    // 初始化通道2 (对应右电机 PWM)
    TIM_OC2Init(TIM4, &TIM_OCInitStructure); 
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); 
    
    // 5. 使能定时器
    TIM_ARRPreloadConfig(TIM4, ENABLE); 
    TIM_Cmd(TIM4, ENABLE); 
}

/**
  * @brief  设置电机速度和方向 (对应图片6)
  * @param  moto1: 左电机速度 (-7200 ~ 7200)
  * @param  moto2: 右电机速度 (-7200 ~ 7200)
  */
void Set_Pwm(int moto1, int moto2)
{
    // --- 处理右电机 (moto2) ---
    // 注意：图片代码中 moto2 对应 AIN/PWMA，moto1 对应 BIN/PWMB
    // 请根据你的实际接线确认左右关系，这里严格按照图片代码逻辑编写
    if(moto2 >= 0) {
        AIN = 0; // 正转方向
        PWMA = myabs(moto2);
    } else {
        AIN = 1; // 反转方向
        PWMA = 7199 - myabs(moto2); // 这里的逻辑是互补PWM或者特定驱动芯片逻辑
    }

    // --- 处理左电机 (moto1) ---
    if(moto1 >= 0) {
        BIN = 0; // 正转方向
        PWMB = myabs(moto1);
    } else {
        BIN = 1; // 反转方向
        PWMB = 7199 - myabs(moto1);
    }
}