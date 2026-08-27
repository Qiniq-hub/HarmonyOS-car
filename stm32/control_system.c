#include "control_system.h"
#include "encoder.h" // 确保包含编码器头文件
#include "motor.h" // 如果你的 Set_Pwm 在 motor.h 里，请包含它

/* 全局变量定义 */
int L_coder, R_coder;      // 左右轮当前编码器读数
int Motor_A, Motor_B;      // 左右轮计算出的 PWM 值
int OverflowTime = 100;    // 采样周期 100ms
volatile uint32_t millis = 0; 

/**
 * @brief 增量式 PI 控制器 (左轮)
 * @param Encoders_A: 当前编码器反馈值
 * @param Target_A: 目标编码器值 (目标速度)
 */
int Incremental_PI_A(int Encoders_A, int Target_A)
{
    float Velocity_KP = 7.0, Velocity_KI = 0.016, Velocity_KD = 0.003; // 参数可根据实际调整
    static int Pwm_A = 0;
    static int Integral_A = 0;
    static float Error_prev_A = 0;
    
    float MaxIntegral = 0.0;
    float MinIntegral = 0.0;

    // 1. 计算偏差
    float Error_A = (float)(Target_A - Encoders_A); 

    // 2. 积分项更新
    Integral_A += Error_A; 

    // 3. 积分限幅 (防止积分饱和)
    MaxIntegral = (float)(7199 / Velocity_KI); 
    MinIntegral = -(float)(7199 / Velocity_KI);
    
    if (Integral_A > MaxIntegral) Integral_A = MaxIntegral;
    else if (Integral_A < MinIntegral) Integral_A = MinIntegral;

    // 4. 计算 PWM 增量 (这里主要用 P 和 I，D项通常很小或不用)
    // 公式：Pwm += Kp * [e(k) - e(k-1)] + Ki * e(k) + Kd * ...
    // 注意：截图中的代码似乎混合了位置式和增量式的写法，这里采用标准的增量式逻辑修正：
    Pwm_A += Velocity_KP * (Error_A - Error_prev_A) + Velocity_KI * Error_A;

    // 5. PWM 限幅 (假设定时器周期是 7199)
    if (Pwm_A > 7199) Pwm_A = 7199;
    else if (Pwm_A < -7199) Pwm_A = -7199;

    // 6. 保存误差供下次计算
    Error_prev_A = Error_A; 

    return Pwm_A;
}

/**
 * @brief 增量式 PI 控制器 (右轮)
 * @note 右轮参数可能需要微调，因为电机个体差异
 */
int Incremental_PI_B(int Encoders_B, int Target_B)
{
    float Velocity_KP = .0, Velocity_KI = 0.016; // 右轮参数
    static int Pwm_B = 0;
    static int Integral_B = 0;
    static float Error_prev_B = 0;
    
    float MaxIntegral = (float)(7199 / Velocity_KI);
    float MinIntegral = -(float)(7199 / Velocity_KI);

    float Error_B = (float)(Target_B - Encoders_B);
    Integral_B += Error_B;

    if (Integral_B > MaxIntegral) Integral_B = MaxIntegral;
    else if (Integral_B < MinIntegral) Integral_B = MinIntegral;

    Pwm_B += Velocity_KP * (Error_B - Error_prev_B) + Velocity_KI * Error_B;

    if (Pwm_B > 7199) Pwm_B = 7199;
    else if (Pwm_B < -7199) Pwm_B = -7199;

    Error_prev_B = Error_B;
    return Pwm_B;
}

/**
 * @brief 转速转换函数 (转/秒 -> 脉冲/100ms)
 */
int Rs_To_CPR(float rads) {
    // 电机ppr: 700, 倍频4. 
    // 公式: rads * (700*4) / (1000 / OverflowTime)
    int CRP = 0;
    CRP = rads * ((700 * 4) / (1000 / OverflowTime));
    return CRP;
}

/**
 * @brief 系统控制主函数 (在中断中调用)
 */
void System_Control(void)
{
    int TageA = 0;
    int TageB = 0;

    // 1. 读取编码器值 (TIM2对应左, TIM3对应右)
    L_coder = Read_Encoder(2);
    R_coder = Read_Encoder(3);

    // 打印调试信息 (可选)
     printf("L:%d R:%d\r\n", L_coder, R_coder);

    // 2. 设定目标速度 (单位：转/秒)
    // 这里设定左右轮都以 1.0 转/秒 运行
    TageA = Rs_To_CPR(1.0);  
    TageB = Rs_To_CPR(1.0); // 如果右轮方向相反，设为负数
    printf("L:%d R:%d\r\n", TageA, TageB);
    // 3. 执行 PID 计算
    Motor_A = Incremental_PI_A(L_coder, TageA);
    Motor_B = Incremental_PI_B(R_coder, TageB);

    // 4. 输出 PWM 到电机
    // 注意：Set_Pwm 函数需要在 motor.c 中实现，用于设置 TIM1/TIM8 的占空比和方向
    Set_Pwm(Motor_A, Motor_B); 
}

/**
 * @brief SysTick 中断服务函数 (每 1ms 进入一次)
 */
void SysTick_Handler(void)
{
    millis++; 
    if (millis % OverflowTime == 0) // 每 100ms 执行一次控制循环
    {
        millis = 0; 
        System_Control(); // 调用上面的控制函数
    }
}