#include "nfc.h" // 确保你有这个头文件，或者把下面的声明放进去
#include "sys.h"
#include "usart.h" 
#include "delay.h"
#include "string.h"
#include "colorful_led.h"   // 用于控制流水灯 R_led_mode / R_led_CLC

// --- 对应图片 5：NFC模块使用的相关指令 ---

// 唤醒命令
u8 const NFC_WakeUp[] = {0x55, 0x55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0x03, 0xFD, 0xD4, 0x14, 0x01, 0x17, 0x00}; 

// 寻卡命令 (查询标签指令)
u8 const NFC_SearchCard[] = {0x00, 0x00, 0xFF, 0x04, 0xFC, 0xD4, 0x4A, 0x01, 0x00, 0xE1, 0x00}; 

// --- 对应图片 5：变量定义区 ---

u8 NFC_WakeUp_Ok = 0;         // NFC唤醒标志
u8 NFC_find_Card = 0;         // NFC找到一张卡
u8 NFC_sendcmd_find = 1;      // NFC收到卡帧头等待
u8 NFC_wait_Card = 0;
u8 NFC_read_id_flag = 0;
u8 NFC_DataBlock[16];         // 存储一个BLOCK的数据

// 串口2接收相关 (假设你的 usart.c 里定义了 UART2Frame，这里引用外部或者重新定义)
// 注意：如果你的 usart.c 已经定义了 UART2Frame，这里不需要再定义结构体，直接用 extern
UART2_FrameTypeDef UART2Frame; 

u8 USART2_RX_BUF[100];         // 接收缓冲
u16 USART2_RX_STA = 0;        // 接收状态标记
u16 slen;                     // 缓冲数组长度
u8 Sys_Stat;                  // nfc id卡状态
u8 Sum = 0;                   // 校验和
u8 REC_LEN = 0;
u8 led_flag = 0;              // LED翻转标志

// --- 函数声明 ---
void FoundCard_Handler(void);
void NFC_user_Handler(void);

// --- 对应图片 9：循环发送寻卡指令 (主循环调用) ---
void NFC_Handler(void)
{
    if(NFC_WakeUp_Ok) // 已唤醒，发指令寻卡
    {
        if(NFC_find_Card == 1) // 是否已寻到卡?
        {
            // 找到一张卡，执行处理函数
            FoundCard_Handler();
        }
        else if(NFC_find_Card == 0 && NFC_sendcmd_find == 1)
        {
            UART2Frame.RxCounter = 0;
            // 未找到卡，发指令
            // 假设 UART2SendFrame 是你的串口发送函数
            // 如果报错，请检查你的 usart.c 中是否有类似 void UART2_Send_Data(u8 *buf, u16 len) 的函数
            UART2SendFrame((u8*)NFC_SearchCard, sizeof(NFC_SearchCard)); 
            
            NFC_sendcmd_find = 0;
            delay_ms(200);
        }
    }
}

// --- 对应图片 7：找到卡执行功能函数 ---
void FoundCard_Handler(void)
{
    NFC_find_Card = 0; // 清除标识
    
    if(led_flag == 0) // 反转车灯
    {
        led_flag = 1;
        R_led_mode(); // 开灯/流水灯 (需确保 led.h 有此函数)
    }
    else
    {
        led_flag = 0;
        R_led_CLC();  // 关灯/清除 (需确保 led.h 有此函数)
    }
    
    NFC_sendcmd_find = 1; // 发送寻卡指令
    delay_ms(200);
}

/**
 * @brief  通过串口2发送一帧数据
 * @param  buf: 指向要发送的数据缓冲区的指针
 * @param  len: 要发送的数据长度（字节数）
 */
void UART2SendFrame(uint8_t *buf, uint16_t len)
{
    uint16_t i;
    
    // 循环发送每一个字节
    for(i = 0; i < len; i++)
    {
        // 1. 等待发送寄存器为空（TXE: Transmit data register empty）
        // 如果上一个字节还没发完，程序会卡在这里等
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        
        // 2. 将当前字节写入发送数据寄存器，硬件会自动发出去
        USART_SendData(USART2, buf[i]);
    }
    
    // 3. 等待最后一个字节真正发送完成（TC: Transmission complete）
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}