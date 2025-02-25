#include "sys.h"

void oled_show(void);
//u16 receive_cnt,receive_cnt2,receive_cnt4;//计算成功接收数据帧次数
u16 distance_1,distance_2,distance_3,distance_4;

u16 receive_cnt2,receive_cnt3,receive_cnt4,receive_cnt5;//计算成功接收数据帧次数
u8 confidence,count;
u16 distance,noise,reftof;
u32 peak,intg;
u8 dis,Send_Flag;
u8 Usart_Send_Data[11];
u8 CAN_Send_Data[8];
int main(void)
{	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断优先级分组，即优先级分级个数
		delay_init();//延时初始化
		JTAG_Set(JTAG_SWD_DISABLE); /*关闭JTAG接口*/   
		JTAG_Set(SWD_ENABLE);    /*打开SWD接口 可以利用主板的SWD接口调试*/       
        LED_Init();
		OLED_Init();
	    CAN1_Mode_Init(1,3,2,6,0);      //=====CAN初始化
		uart_init(115200);//串口1初始化
		uart2_init(230400);//串口接收
		uart3_init(230400);//串口接收
		uart4_init(230400);//串口接收
		uart5_init(230400);//串口接收
        time5_init(); //定时器中断，处理数据
		while(1)
		{	
				
				oled_show();//oled显示
				if(Send_Flag==1)//20HZ发送数据
				{
					Send_Flag=0;
					data_trastion();
					USART1_SEND(); 
					CAN1_Send_Num(0x601,CAN_Send_Data);
				}
		}
}



