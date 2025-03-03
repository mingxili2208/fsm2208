#include "a69.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "24l01.h"
#include <stdio.h>
#include "ax_robot.h"
#include "ax_oled.h"

// **接收缓冲区**
static uint8_t rxBuffer[A69_RX_BUFFER_SIZE];
static uint16_t rxWriteIndex = 0;
static uint16_t rxReadIndex = 0;

extern u16 distance_2,distance_3;    

// **GPIO 配置**
static void A69_GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // **使能 GPIOB, GPIOC, GPIOD 时钟**
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    // **配置 MD0 和 MD1 为推挽输出**
    GPIO_InitStructure.GPIO_Pin = A69_MD0_PIN | A69_MD1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(A69_MD0_PORT, &GPIO_InitStructure);

    // **配置 AUX 为输入**
    GPIO_InitStructure.GPIO_Pin = A69_AUX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // **下拉输入**
    GPIO_Init(A69_AUX_PORT, &GPIO_InitStructure);

    // **配置 UART5 TX (PC12) 和 RX (PD2)**
    // **TX (PC12) - 复用推挽输出**
    GPIO_InitStructure.GPIO_Pin = A69_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(A69_TX_PORT, &GPIO_InitStructure);

    // **RX (PD2) - 浮空输入**
    GPIO_InitStructure.GPIO_Pin = A69_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(A69_RX_PORT, &GPIO_InitStructure);
}

// **配置 UART5**
static void A69_UART_Config(void) {
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // **使能 UART5 时钟**
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    // **配置 UART5 参数**
    USART_InitStructure.USART_BaudRate = A69_BAUD_RATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(UART5, &USART_InitStructure);

    // **使能接收中断**
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

    // **配置 NVIC**
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // **使能 UART5**
    USART_Cmd(UART5, ENABLE);
}

// **A69 初始化**
void A69_Init(void) {
    A69_GPIO_Config();
    A69_UART_Config();
}

// **发送数据**
void A69_SendData(uint8_t *data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
        USART_SendData(UART5, data[i]);
    }
}

// **UART5 接收中断处理**
void UART5_IRQHandler(void) {
	
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
				
        uint8_t receivedByte = USART_ReceiveData(UART5);

        // **计算下一个写入位置**
        uint16_t nextIndex = (rxWriteIndex + 1) % A69_RX_BUFFER_SIZE;

        // **如果缓冲区未满，才写入**
		if (nextIndex != rxReadIndex) {
				rxBuffer[rxWriteIndex] = receivedByte;
				rxWriteIndex = nextIndex;
		} else {
				AX_OLED_DispStr(0, 1, " buff_overwrite!! ", 0);
				
				// **覆盖最旧的数据**
				rxReadIndex = (rxReadIndex + 1) % A69_RX_BUFFER_SIZE; 
				rxBuffer[rxWriteIndex] = receivedByte;
				rxWriteIndex = nextIndex;
		}
    }
}

// **检查是否有数据可读**
uint8_t A69_DataAvailable(void) {
    return (rxWriteIndex != rxReadIndex);
}

// **读取数据**
uint8_t A69_ReceiveData(uint8_t *data, uint16_t maxLength) {
    uint16_t i = 0;

    // **确保从帧头 `0x55 0x7E` 开始**
    while (rxWriteIndex != rxReadIndex) {
        if (rxBuffer[rxReadIndex] == 0x55 && rxBuffer[(rxReadIndex + 1) % A69_RX_BUFFER_SIZE] == 0x7E) {
            break; // 找到帧头
        }
        rxReadIndex = (rxReadIndex + 1) % A69_RX_BUFFER_SIZE; // 丢弃无效字节
    }

    // **读取完整的数据帧**
    while (i < maxLength && rxWriteIndex != rxReadIndex) {
        data[i++] = rxBuffer[rxReadIndex];
        rxReadIndex = (rxReadIndex + 1) % A69_RX_BUFFER_SIZE;
    }

    return i; // 返回读取的数据长度
}

// **设置 A69 模式**
void A69_SetMode(uint8_t md0_state, uint8_t md1_state) {
    GPIO_WriteBit(A69_MD0_PORT, A69_MD0_PIN, (BitAction)md0_state);
    GPIO_WriteBit(A69_MD1_PORT, A69_MD1_PIN, (BitAction)md1_state);
}

void FSM_A69_ProcessReceivedData(uint8_t *data, uint8_t length, NRF_CTL_INFO *nrt_ctl_info) {
    if (length < 10) return; // 确保数据长度足够

    // **检查帧头和帧尾**
    if (!(data[0] == 0x55 && data[1] == 0x7E && data[8] == 0x7E && data[9] == 0x55)) {
				FSM_A69_SendRec(0x00);
        return; // 帧格式错误，丢弃数据
    }

    // **校验 XOR 校验位**
    uint8_t checksum = data[3] ^ data[4] ^ data[5] ^ data[6];
    if (checksum != data[7]) {
				FSM_A69_SendRec(checksum);
				FSM_A69_SendTest(data[3],data[4],data[5],data[6]);
        return; // 校验失败，丢弃数据
    }

    // **解析数据**
    nrt_ctl_info->ST = data[2];
		if(nrt_ctl_info->ST==0x01){
				nrt_ctl_info->steering_angle = data[3];
				nrt_ctl_info->speed = data[5];
			FSM_A69_SendSpeed(R_Vel.TG_IX,ax_akm_angle);
		}
		//FSM_A69_SendFAS(nrt_ctl_info->ST,nrt_ctl_info->steering_angle,nrt_ctl_info->speed);
		
    // **显示数据到 OLED**
//    char hex_str[25];
//    snprintf(hex_str, sizeof(hex_str), " ST:%02X SPD:%02X ANG:%02X ", nrt_ctl_info->ST, nrt_ctl_info->speed, nrt_ctl_info->steering_angle);
//    AX_OLED_DispStr(0, 1, (uint8_t*)hex_str, 0);
}
void FSM_A69_SendDis(void)
{
    uint8_t tx_buf[10];

    tx_buf[0] = 0x55;
    tx_buf[1] = 0x7E;
    tx_buf[2] = 0x03;  // 车发送数据标志
    tx_buf[3] = (distance_2 >> 8) & 0xFF; // 高字节
    tx_buf[4] = distance_2 & 0xFF;        // 低字节
    tx_buf[5] = (distance_3 >> 8) & 0xFF; // 高字节
    tx_buf[6] = distance_3 & 0xFF;        // 低字节
    tx_buf[7] = tx_buf[3] ^ tx_buf[4] ^ tx_buf[5] ^ tx_buf[6]; // 校验位
    tx_buf[8] = 0x7E;
    tx_buf[9] = 0x55;

    // 通过 A69 发送数据
    A69_SendData(tx_buf, sizeof(tx_buf));
}
void FSM_A69_SendSpeed(uint16_t data1,uint16_t data2)
{
    uint8_t tx_buf[10];

    tx_buf[0] = 0x55;
    tx_buf[1] = 0x7E;
    tx_buf[2] = 0x01;  // 车发送数据标志
    tx_buf[3] = (data1 >> 8) & 0xFF; // 高字节
    tx_buf[4] = data1 & 0xFF;        // 低字节
    tx_buf[5] = (data2 >> 8) & 0xFF; // 高字节
    tx_buf[6] = data2 & 0xFF;        // 低字节
    tx_buf[7] = tx_buf[3] ^ tx_buf[4] ^ tx_buf[5] ^ tx_buf[6]; // 校验位
    tx_buf[8] = 0x7E;
    tx_buf[9] = 0x55;

    // 通过 A69 发送数据
    A69_SendData(tx_buf, sizeof(tx_buf));
}

void FSM_A69_SendRec(uint8_t flag)
{
    uint8_t tx_buf[10];

    tx_buf[0] = 0x55;
    tx_buf[1] = 0x7E;
    tx_buf[2] = flag;  // 车发送数据标志
    tx_buf[3] = 0x00; // 高字节
    tx_buf[4] = 0x00;        // 低字节
    tx_buf[5] = 0x00; // 高字节
    tx_buf[6] = 0x00;        // 低字节
    tx_buf[7] = tx_buf[3] ^ tx_buf[4] ^ tx_buf[5] ^ tx_buf[6]; // 校验位
    tx_buf[8] = 0x7E;
    tx_buf[9] = 0x55;

    // 通过 A69 发送数据
    A69_SendData(tx_buf, sizeof(tx_buf));
}
void FSM_A69_SendFAS(uint8_t flag,uint8_t angle ,uint8_t speed )
{
    uint8_t tx_buf[10];

    tx_buf[0] = 0x55;
    tx_buf[1] = 0x7E;
    tx_buf[2] = flag;  // 车发送数据标志
    tx_buf[3] = angle; // 高字节
    tx_buf[4] = 0x00;        // 低字节
    tx_buf[5] = speed; // 高字节
    tx_buf[6] = 0x00;        // 低字节
    tx_buf[7] = tx_buf[3] ^ tx_buf[4] ^ tx_buf[5] ^ tx_buf[6]; // 校验位
    tx_buf[8] = 0x7E;
    tx_buf[9] = 0x55;

    // 通过 A69 发送数据
    A69_SendData(tx_buf, sizeof(tx_buf));
}
void FSM_A69_SendTest(uint8_t a,uint8_t b ,uint8_t c, uint8_t d )
{
    uint8_t tx_buf[10];

    tx_buf[0] = 0x55;
    tx_buf[1] = 0x7E;
    tx_buf[2] = 0x11;  // 车发送数据标志
    tx_buf[3] = a; // 高字节
    tx_buf[4] = b;        // 低字节
    tx_buf[5] = c; // 高字节
    tx_buf[6] = d;        // 低字节
    tx_buf[7] = tx_buf[3] ^ tx_buf[4] ^ tx_buf[5] ^ tx_buf[6]; // 校验位
    tx_buf[8] = 0x7E;
    tx_buf[9] = 0x55;

    // 通过 A69 发送数据
    A69_SendData(tx_buf, sizeof(tx_buf));
}