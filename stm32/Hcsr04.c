#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_watchdog.h"
#include "hi_io.h"
#include "hi_time.h"

/* (3) 定义相关变量 */
// HC-SR04 超声波测距模块通过 GPIO7 和 GPIO8 连接到 3861
#define GPIO_8 8
#define GPIO_7 7
#define GPIO_FUNC 0
#define IoTGpioSetDir GpioSetDir

/* (4) 任务书写 - 测距功能实现 */
float GetDistance(void)
{
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0;

    hi_io_set_func(GPIO_8, GPIO_FUNC);

    // GPIO_8 设置为输入引脚 (Echo)
    GpioSetDir(GPIO_8, WIFI_IOT_GPIO_DIR_IN);
    // GPIO_7 设置为输出引脚 (Trig)
    GpioSetDir(GPIO_7, WIFI_IOT_GPIO_DIR_OUT);

    // GPIO_7 输出一个脉冲触发信号到超声波测距模块，至少 10us
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE0);

    // 超声波测距模块接收到 GPIO_7 输出的脉冲触发信号后，模块输出回响信号（高电平）到 GPIO_8
    while (1) {
        GpioGetInputVal(GPIO_8, &value);

        // 测量回响信号（高电平）时间
        // 检测到高电平上升沿，记录开始时间
        if (value == WIFI_IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }

        // 检测到低电平下降沿，计算时间差并退出循环
        if (value == WIFI_IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            start_time = 0;
            break;
        }
    }

    // 距离 = 高电平时间 * 0.034 / 2 (声速 340m/s -> 0.034cm/us)
    distance = time * 0.034 / 2;
    return distance;
}

/* 线程执行函数 */
void Hcsrtext(void* parame) {
    (void)parame;
    printf("start test hcsr04\r\n");

    // 重复执行测距功能，测量周期为 2s
    while (1) {
        float distance = GetDistance();
        printf("distance is %.1f (cm)\r\n", distance);
        osDelay(200);
    }
}

/* 任务入口 */
static void Hcsr04(void)
{
    WatchDogDisable(); // 关闭看门狗
    osThreadAttr_t attr;

    attr.name = "Hcsr04";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = osPriorityNormal;

    // 注意：这里调用了 Hcsrtext 函数，该函数定义在下方
    if (osThreadNew(Hcsrtext, NULL, &attr) == NULL) {
        printf("Failed to create Task!\n");
    }
}



/* (5) 启动任务（添加在整个文件的最末尾） */
APP_FEATURE_INIT(Hcsr04); // 任务启动