#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"

// 定义引脚宏定义 (根据图片中的代码逻辑推断)
// AIN/BIN 是方向控制引脚
#define AIN PBout(14) // 对应图片中的 GPIO_Pin_14
#define BIN PBout(13) // 对应图片中的 GPIO_Pin_13

// PWMA/PWMB 是PWM输出通道 (对应 TIM4 的 CH1 和 CH4)
// 这里直接操作寄存器来修改占空比
#define PWMA TIM4->CCR1
#define PWMB TIM4->CCR4

void Motor_Init(void);      // 初始化电机方向引脚
void PWM_Init(u16 arr, u16 psc); // 初始化定时器PWM
void Set_Pwm(int moto1, int moto2); // 设置电机速度和方向

#endif