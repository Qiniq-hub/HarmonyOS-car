#include "sys.h"
#include "usart.h"	  
#include "nfc.h"
void put_HEX(USART_TypeDef* USARTx, uint8_t *buf, uint16_t len);


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 


#if EN_USART1_RX   //如果使能了接收

u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 USART_RX_STA=0;       //接收状态标记	  
u8 count=0;
u8 USART_RX_COUNT=0;
u8 uart_rec_flag=0;
u8 CAR_buff[4]={0};
void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

void USART1_IRQHandler(void)                 //串口1中断服务程序
{
    u8 Res;
    
    // 判断是否是接收中断
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
    {
        Res = USART_ReceiveData(USART1); // 读取接收到的数据
        
        // --- 【核心修改】这里采用了图一的“寻找帧头”逻辑 ---
        
        // 1. 判断当前收到的字节是不是帧头 (假设帧头是 0xFC，如果是 0x01 请自行修改)
        if(Res == 0xFC) 
        {
            USART_RX_BUF[0] = Res;   // 存入缓冲区第一位
            USART_RX_COUNT = 1;      // 计数器置为 1 (因为已经收到1个了)
        }
        else
        {
            // 如果不是帧头，就继续往后存
            if(USART_RX_COUNT > 0) // 只有当已经开始接收(找到帧头)后才存数据
            {
                USART_RX_BUF[USART_RX_COUNT] = Res;
                USART_RX_COUNT++;
            }
        }
        
        // 2. 判断是否收到了一整帧数据 (假设一帧总共 6 个字节)
        // 图一逻辑：当计数器达到 6，说明收完了 [0]到[5]
        if(USART_RX_COUNT >= 6) 
        {
            // 3. 校验帧尾 (图一逻辑：检查第6个字节是不是 0xFD)
            if(USART_RX_BUF[5] == 0xFD) 
            {
                // --- 【核心修改】这里采用了图一的“数据提取”逻辑 ---
                
                // 将原始缓冲区的数据搬运到 CAR_buff
                CAR_buff[0] = USART_RX_BUF[1]; // 方向A
                CAR_buff[1] = USART_RX_BUF[2]; // 电机A速度
                CAR_buff[2] = USART_RX_BUF[3]; // 方向B
                CAR_buff[3] = USART_RX_BUF[4]; // 电机B速度
                
                // 设置接收完成标志
                uart_rec_flag = 1; 
                
                // 打印调试信息 (可选)
                // printf("收到完整帧: %x %x %x %x\r\n", CAR_buff[0], CAR_buff[1], CAR_buff[2], CAR_buff[3]);
            }
            
            // 无论校验成功与否，都清零计数器，准备接收下一帧
            USART_RX_COUNT = 0; 
        }
    }
    
    // 清除中断标志位 (根据你的库版本，有时候不需要这句，如果报错可注释掉)
    // USART_ClearFlag(USART1, USART_FLAG_RXNE); 
}

void USART2_IRQHandler(void)
{
    // 判断是否是接收中断
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
    {
        // 1. 读取接收到的字节存入缓冲区
        UART2Frame.RxBuffer[UART2Frame.RxCounter] = USART_ReceiveData(USART2);

        // --- 分支一：未唤醒状态 ---
        if(NFC_WakeUp_Ok == 0) 
        {
            UART2Frame.RxCounter++;
            
            // 图片逻辑：如果接收到了15个字节（唤醒响应长度）
            if(UART2Frame.RxCounter >= 15) 
            {
                // 将数据复制到全局缓冲区 USART2_RX_BUF
                memcpy(USART2_RX_BUF, (uint8_t*)UART2Frame.RxBuffer, 15);
                
                // 清空局部缓冲区和计数器
                memset((uint8_t*)UART2Frame.RxBuffer, 0, 20);
                UART2Frame.RxCounter = 0;
                
                // 注意：这里通常应该置位 NFC_WakeUp_Ok = 1; 
                // 但图片没显示这一步，如果你的主循环里没有置位，记得加上，否则会一直卡在这里
            }
        }
        
        // --- 分支二：已唤醒，进入寻卡流程 ---
        else 
        {
            UART2Frame.RxCounter++;

            // 图片逻辑：如果接收到了25个字节（寻卡响应长度）
            if(UART2Frame.RxCounter >= 25) 
            {
                // 复制数据
                memcpy(USART2_RX_BUF, (uint8_t*)UART2Frame.RxBuffer, 25);
                
                // 打印调试信息 (put_HEX 是你工程里的函数)
                put_HEX(USART1, USART2_RX_BUF, 25);

                // 2. 校验卡片类型 (UID的前几位或特定标识)
                // 图片中的长判断条件还原：
                if(
                    ((0xB9 == USART2_RX_BUF[19]) && (0x80 == USART2_RX_BUF[20]) && (0x06 == USART2_RX_BUF[21]) && (0x85 == USART2_RX_BUF[22]))
                    ||
                    ((0x50 == USART2_RX_BUF[19]) && (0x84 == USART2_RX_BUF[20]) && (0xFC == USART2_RX_BUF[21]) && (0x23 == USART2_RX_BUF[22]))
                    ||
                    ((0x40 == USART2_RX_BUF[19]) && (0x74 == USART2_RX_BUF[20]) && (0x80 == USART2_RX_BUF[21]) && (0x23 == USART2_RX_BUF[22]))
                )
                {
                    NFC_find_Card = 1; // 标记找到卡片
                }

                // 清空缓冲区，准备下一次接收
                memset((uint8_t*)UART2Frame.RxBuffer, 0, 50);
                memset((uint8_t*)USART2_RX_BUF, 0, 50);
                UART2Frame.RxCounter = 0;
            }
        }
    }
}

void put_HEX(USART_TypeDef* USARTx, uint8_t *buf, uint16_t len)
{
    uint16_t i,j;
    char temp_str[5]; // 用于存放转换后的字符串 "XX "
   
    for(i = 0; i < len; i++)
    {
        // 将字节转换为十六进制字符串，例如 0x1A -> "1A "
        sprintf(temp_str, "%02X ", buf[i]); 
        
        // 逐个字符发送
       
        while(temp_str[j] != '\0')
        {
            // 等待发送缓冲区为空
            while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); 
            // 发送字符
            USART_SendData(USARTx, temp_str[j]); 
            j++;
        }
    }
    // 发送一个换行符，方便查看
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
    USART_SendData(USARTx, '\n');
}


#endif	

