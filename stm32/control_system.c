#include "control_system.h"
#include "usart.h"
// #include "sys.h" // 根据需要包含
// typedef enum {false = 0, true = 1} bool; // 如果需要bool类型可取消注释

/*电机 左A 右B*/
int L_coder, R_coder;

int Motor_A, Motor_B;       //电机PWM变量
int OverflowTime = 100;
volatile uint32_t millis = 0; // 记录毫秒数
volatile uint32_t seconds = 0; // 记录秒数

/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回 值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差 以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PD控制
pwm+=KD[e(k)-e(k-1)]+Kp*e(k)
**************************************************************************/
int Incremental_PI_A(int Encoders_A, int Target_A)
{
    float Velocity_KP = 7.0, Velocity_KI = 0.016, Velocity_KD = 0.003;
    static int Pwm_A = 0;
    static int Integral_A = 0;
    static float Error_prev_A = 0;
    
    float MaxIntegral = 0.0;
    float MinIntegral = 0.0;
    
    float Error_A = (float)(Target_A - Encoders_A); // 计算偏差

    Integral_A += Error_A; // 积分项更新

    // 积分限幅
    MaxIntegral = (float)(7199 / Velocity_KI);
    MinIntegral = -(float)(7199 / Velocity_KI);
    if (Integral_A > MaxIntegral) Integral_A = MaxIntegral;
    else if (Integral_A < MinIntegral) Integral_A = MinIntegral;

    Pwm_A += Velocity_KP * Error_A + Velocity_KD * (Error_A - Error_prev_A);

    if (Pwm_A > 7199) Pwm_A = 7199;
    else if (Pwm_A < -7199) Pwm_A = -7199;

    Error_prev_A = Error_A; // 保存上一次偏差

    return Pwm_A; // 增量输出
}

int Incremental_PI_B(int Encoders_B, int Target_B)
{
    // 参数与Incremental_PI_A中不一致，因电机的批次或者安装的影响，阻力不同，需要不同的P值
    // 电机阻力越大，P值相应的增大一些，保持两个电机几乎同时达到目标转速
    // 若追求效果可以调整I和D
    float Velocity_KP = 7.0, Velocity_KI = 0.016, Velocity_KD = 0.003;
    
    static int Pwm_B = 0;
    static int Integral_B = 0;
    static float Error_prev_B = 0;
    
    float MaxIntegral = 0.0;
    float MinIntegral = 0.0;
    
    float Error_B = (float)(Target_B - Encoders_B); // 计算偏差

    Integral_B += Error_B; // 积分项更新

    // 积分限幅
    MaxIntegral = (float)(7199 / Velocity_KI);
    MinIntegral = -(float)(7199 / Velocity_KI);
    if (Integral_B > MaxIntegral) Integral_B = MaxIntegral;
    else if (Integral_B < MinIntegral) Integral_B = MinIntegral;

    Pwm_B += Velocity_KP * Error_B + Velocity_KD * (Error_B - Error_prev_B);

    if (Pwm_B > 7199) Pwm_B = 7199;
    else if (Pwm_B < -7199) Pwm_B = -7199;

    Error_prev_B = Error_B; // 保存上一次偏差

    return Pwm_B; // 增量输出
}

/**************************************************************************
函数功能：转每秒转脉冲数函数
入口参数：float
返回 值：int

电机ppr: 700，倍频4
设定电机转速为1转/s，已知电机1转产生(700*4)脉冲，则每100ms产生的脉冲数为：(700*4)/(1000/100)，单位：脉冲数/100ms
**************************************************************************/
int Rs_To_CPR(float rads){ // rads取值范围：-1.5 ~ 1.5，即最大设定转速为1.5转/s
    int CRP = 0;
    CRP = rads * ((700*4)/(1000/OverflowTime));
    return CRP;
}

/**************************************************************************
函数功能：系统控制函数
入口参数：
返回 值：
**************************************************************************/
extern u8 uart_rec_flag;
extern u8 CAR_buff[4];
void System_Control(void)
{
    //理论编码器值 (实际速度反馈)
    int TageA = 0; 
    int TageB = 0; 
    
    // 定义目标速度变量 (对应你第一张图的逻辑)
    // 注意：这里假设 Target_MotorA/B 是全局变量，或者你在函数内部定义 float Target_MotorA, Target_MotorB;
    float Target_MotorA = 0; 
    float Target_MotorB = 0;

    /********* 1. 获取解析数据帧 (来自第一张图) *********/
    if(uart_rec_flag)  // 收到一帧数据
    {
        // --- 解析速度绝对值 ---
        // 假设 CAR_buff[1] 和 [3] 是放大100倍后的整数，这里还原成浮点数
        Target_MotorA = CAR_buff[1] / 100.00; 
        Target_MotorB = CAR_buff[3] / 100.00; 

        // --- 转速值还原 (处理方向/正负) ---
        if(CAR_buff[0] == 1){ // 假设 buff[0] 是左轮方向标志位
            Target_MotorA = -1 * Target_MotorA;
        }
        if(CAR_buff[2] == 1){ // 假设 buff[2] 是右轮方向标志位
            Target_MotorB = -1 * Target_MotorB;
        }

        // --- 倒车灯逻辑 ---
        if(CAR_buff[0]==1 && CAR_buff[2]==1){
            R_led_mode(); // 开倒车灯
        } else {
            R_led_CLC();  // 关倒车灯
        }

        // --- 清除标志位和缓冲区 ---
        uart_rec_flag = 0;
        memset(CAR_buff, 0, 4); 
    }
    /**********************************************/


    //读取 OverflowTime ms时间的脉冲数 (来自第二张图)
    L_coder = Read_Encoder(2);
    R_coder = Read_Encoder(3);

    //打印调试信息 (可选)
    // printf("left coder : %d\r\n", L_coder);
    // printf("Target A : %.2f\r\n", Target_MotorA); // 建议打印看看目标值对不对

    //计算 OverflowTime 时间转每秒的速度应达到的编码器值
    // 【关键修改点】：不再使用 Rs_To_CPR(1.0)，而是使用上面解析出来的 Target_MotorA
    // 注意：如果你的 PID 输入需要是编码器脉冲数，可能需要把 Target_MotorA 也转换一下单位
    // 如果 Target_MotorA 已经是“转/秒”，则保持原样；如果是其他单位，需自行换算。
    
    // 假设你的 Rs_To_CPR 函数是将 "转/秒" 转换为 "周期内的脉冲数"
    // 那么这里应该这样写：
    TageA = Rs_To_CPR(Target_MotorA); 
    TageB = Rs_To_CPR(Target_MotorB);


    //速度闭环控制计算机电机最终 PWM
    // 将计算出的目标脉冲数(TageA)和实际脉冲数(L_coder)传入 PID
    Motor_A = Incremental_PI_A(L_coder, TageA); 
    Motor_B = Incremental_PI_B(R_coder, TageB);

    //设置电机转速
    Set_Pwm(Motor_A, Motor_B);
}

/**
 * @brief 系统滴答定时器中断服务函数
 * @param None
 * @retval : None
 */
void SysTick_Handler(void)
{
    millis++; // 每次滴答定时器中断，毫秒数加1
    if (millis % OverflowTime == 0) // 如果毫秒数达到100
    {
        millis = 0; // 毫秒数清零
        //seconds++; // 秒数加1
        
        System_Control();
    }
}




