#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include "ohos_init.h"
#include "cmsis_os2.h"

#include "wifiiot_uart.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_watchdog.h"
#include "hi_io.h"
#include "hi_time.h"

// 1. 红外巡线传感器引脚定义 (GPIO_13: 左红外, GPIO_14: 右红外)
#define GPIOL WIFI_IOT_IO_NAME_GPIO_13
#define GPIOR WIFI_IOT_IO_NAME_GPIO_14

// 2. 超声波传感器引脚定义
#define TRIG_GPIO 7
#define ECHO_GPIO 8
#define GPIO_FUNC 0

static uint8_t uart_sendbuf[6];

/******************* 串口硬件初始化 *******************/

void stm32_uart_init(void)
{
    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);

    WifiIotUartAttribute uart_attr2 = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);
}

/******************* 超声波 HC-SR04 驱动 *******************/

static void Hcsr04_GpioInit(void)
{
    hi_io_set_func(ECHO_GPIO, GPIO_FUNC);
    GpioSetDir(ECHO_GPIO, WIFI_IOT_GPIO_DIR_IN);

    hi_io_set_func(TRIG_GPIO, GPIO_FUNC);
    GpioSetDir(TRIG_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);
}

float GetDistance(void)
{
    uint32_t start_time = 0;
    uint32_t duration = 0;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    uint32_t timeout = 0;

    GpioSetOutputVal(TRIG_GPIO, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);

    timeout = 0;
    while (1) {
        GpioGetInputVal(ECHO_GPIO, &value);
        if (value == WIFI_IOT_GPIO_VALUE1) {
            start_time = hi_get_us();
            break;
        }
        hi_udelay(1);
        timeout++;
        if (timeout > 10000) {
            return -1.0f;
        }
    }

    timeout = 0;
    while (1) {
        GpioGetInputVal(ECHO_GPIO, &value);
        if (value == WIFI_IOT_GPIO_VALUE0) {
            duration = hi_get_us() - start_time;
            break;
        }
        hi_udelay(1);
        timeout++;
        if (timeout > 30000) {
            return -2.0f;
        }
    }

    return ((float)duration * 0.034f) / 2.0f;
}

/******************* 电机控制协议与基础动作 *******************/

void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = (motorA < 0) ? 1 : 0;
    uint8_t B_dir = (motorB < 0) ? 1 : 0;

    int speedA = abs(motorA);
    int speedB = abs(motorB);

    if (speedA > 150) speedA = 150;
    if (speedB > 150) speedB = 150;

    uart_sendbuf[0] = 0xFC;
    uart_sendbuf[1] = A_dir;
    uart_sendbuf[2] = (uint8_t)speedA;
    uart_sendbuf[3] = B_dir;
    uart_sendbuf[4] = (uint8_t)speedB;
    uart_sendbuf[5] = 0xFD;

    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

void car_forward(void)  { stm32motor_control(60, 60); }
void car_backward(void) { stm32motor_control(-60, -60); }
void car_left(void)     { stm32motor_control(-50, 50); }  // 原地左转
void car_right(void)    { stm32motor_control(50, -50); }  // 原地右转
void car_stop(void)     { stm32motor_control(0, 0); }

void car_emergency_brake(void)
{
    stm32motor_control(-80, -80);
    usleep(100000); // 100ms 强反推
    car_stop();
}

/******************* 综合避障与计数巡线控制任务 *******************/

static void VehicleControlTask(void)
{
    WatchDogDisable();

    stm32_uart_init();
    
    IoSetFunc(GPIOL, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(GPIOR, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(GPIOL, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(GPIOR, WIFI_IOT_GPIO_DIR_IN);

    Hcsr04_GpioInit();

    printf("Vehicle Control & Intersection Counter Task Started!\r\n");

    WifiIotGpioValue status_l = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue status_r = WIFI_IOT_GPIO_VALUE0;
    
    // 岔路/横线计数器
    uint32_t double_line_count = 0;

    while (1) {
        // 读取传感器状态
        float distance = GetDistance();
        GpioGetInputVal(GPIOL, &status_l);
        GpioGetInputVal(GPIOR, &status_r);

        // 优先级 1：前方障碍物避障 (< 20cm)
        if (distance > 0.0f && distance < 20.0f) {
            printf("Obstacle Detected! Distance: %.1f cm -> Action: Avoid Obstacle\r\n", distance);
            
            car_emergency_brake();
            usleep(100000);
            
            car_backward();
            usleep(400000);  // 倒车 0.4s
            
            car_right();
            usleep(800000);  // 右转避障
        } 
        // 优先级 2：左右两边同时检测到黑线 -> 触发计数器判断
        else if (status_l == WIFI_IOT_GPIO_VALUE1 && status_r == WIFI_IOT_GPIO_VALUE1) {
            double_line_count ^= 1;
            printf("Double Line Detected! Current Count: %u\r\n", double_line_count);

            // 奇数：左转
            if (double_line_count) {
                printf("Count is ODD (%u) -> Action: Turn Left\r\n", double_line_count);
                car_left();
                usleep(600000); // 转向持续时间（可根据实际车速和转弯角度微调）
            } 
            // 偶数：右转
            else {
                printf("Count is EVEN (%u) -> Action: Turn Right\r\n", double_line_count);
                car_right();
                usleep(600000); // 转向持续时间（可根据实际车速和转弯角度微调）
            }

            // 【防重复计数/消抖】：转向后短暂直行冲出当前双黑线交汇区域
            car_forward();
            usleep(200000); 
        }
        // 优先级 3：偏右（仅左红外检测到黑线 -> 左转微调）
        else if (status_l == WIFI_IOT_GPIO_VALUE1 && status_r == WIFI_IOT_GPIO_VALUE0) {
            car_left();
        }
        // 优先级 4：偏左（仅右红外检测到黑线 -> 右转微调）
        else if (status_l == WIFI_IOT_GPIO_VALUE0 && status_r == WIFI_IOT_GPIO_VALUE1) {
            car_right();
        }
        // 优先级 5：双白线上，沿线直行
        else {
            car_forward();
        }

        // 轮询延时 (10ms)
        usleep(10000); 
    }
}

/******************* 入口线程注册 *******************/

void RobotCarInit(void)
{
    osThreadAttr_t attr = {0};
    attr.name = "VehicleControlTask";
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)VehicleControlTask, NULL, &attr) == NULL) {
        printf("Failed to create VehicleControlTask!\r\n");
    }
}

APP_FEATURE_INIT(RobotCarInit);
