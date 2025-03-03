#ifndef A69_H
#define A69_H

#include "stm32f10x.h"
#include "24l01.h"

// 配置 UART5
#define A69_BAUD_RATE   9600 // 波特率
#define A69_UART        UART5   // 选择 UART5
#define A69_IRQn        UART5_IRQn // UART5 中断号

// **MD0 和 MD1 控制引脚 (用于模式切换)**
#define A69_MD0_PORT    GPIOB
#define A69_MD0_PIN     GPIO_Pin_14
#define A69_MD1_PORT    GPIOB
#define A69_MD1_PIN     GPIO_Pin_13

// **AUX 状态引脚**
#define A69_AUX_PORT    GPIOB
#define A69_AUX_PIN     GPIO_Pin_12

// **UART5 引脚 (TX: PC12, RX: PD2)**
#define A69_TX_PORT     GPIOC
#define A69_TX_PIN      GPIO_Pin_12
#define A69_RX_PORT     GPIOD
#define A69_RX_PIN      GPIO_Pin_2

// **接收缓冲区大小**
#define A69_RX_BUFFER_SIZE 128

// **函数声明**
void A69_Init(void);
void A69_SendData(uint8_t *data, uint16_t length);
uint8_t A69_ReceiveData(uint8_t *data, uint16_t maxLength);
void A69_SetMode(uint8_t md0_state, uint8_t md1_state);
uint8_t A69_DataAvailable(void);
void FSM_A69_ProcessReceivedData(uint8_t *data, uint8_t length,NRF_CTL_INFO *nrt_ctl_info);
void FSM_A69_SendDis(void);
void FSM_A69_SendRec(uint8_t flag);
void FSM_A69_SendFAS(uint8_t flag,uint8_t angle ,uint8_t speed );
void FSM_A69_SendTest(uint8_t a,uint8_t b ,uint8_t c, uint8_t d );
void FSM_A69_SendSpeed(uint16_t data1,uint16_t data2);
#endif