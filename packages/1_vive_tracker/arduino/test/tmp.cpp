#include "24l01.h"
#include "spi.h"
#include "ax_beep.h"
#include <stm32f10x.h>
#include "ax_delay.h"
#include "ax_led.h"
#include "ax_oled.h"


    
const u8 TX_ADDRESS[TX_ADR_WIDTH]={0xff,0xff,0xff,0xff,0xff}; //send_address
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0xff,0xff,0xff,0xff,0xff};	//receive_address

u8 NRF_beep_flag=3;	//nrfÅäÖÃµÄbeepµÄflag   1 ½ÓÊÜ³É¹¦£¬2½ÓÊÜÊ§°Ü

u8 NRF_led_flag=3;	//nrf½ÓÊÜ³É¹¦Ôòflag ÖÃ1 ÂÌµÆ¶ÌÁÁ£¬·ñÔòÖÃ0 ºìµÆ¶ÌÁÁ

//³õÊ¼»¯24L01µÄIO¿Ú
void NRF24L01_Init(void)
{ 
	
//	AX_BEEP_On();
//	AX_Delayms(1000);	
//	AX_BEEP_Off();
//	AX_Delayms(1000);
	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOC, ENABLE);	 //Ê¹ÄÜPB,D,C¶Ë¿ÚÊ±ÖÓ
    	
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PB12ÉÏÀ­ ·ÀÖ¹W25XµÄ¸ÉÈÅ
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //ÍÆÍìÊä³ö
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);	//³õÊ¼»¯Ö¸¶¨IO
// 	GPIO_SetBits(GPIOB,GPIO_Pin_12);//ÉÏÀ­	
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA1 ÊäÈë  
	GPIO_Init(GPIOC, &GPIO_InitStructure);			 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	//PA0 ÍÆÍì 	  
 	GPIO_Init(GPIOD, &GPIO_InitStructure);//³õÊ¼»¯Ö¸¶¨IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	//PB12 ÍÆÍì 	  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//³õÊ¼»¯Ö¸¶¨IO

//	AX_BEEP_On();
//	AX_Delayms(100);	
//	AX_BEEP_Off();
//	AX_Delayms(1000);
//	GPIO_ResetBits(GPIOD,GPIO_Pin_2);// Pull-up
//	GPIO_ResetBits(GPIOB,GPIO_Pin_11);// Pull-up
//	GPIO_ResetBits(GPIOC,GPIO_Pin_12);// Pull-up
	
		 
  SPI2_Init();    		//³õÊ¼»¯SPI	 
	
//	AX_BEEP_On();
//	AX_Delayms(100);	
//	AX_BEEP_Off();
//	AX_Delayms(1000);
	SPI_Cmd(SPI2, DISABLE); // SPIÍâÉè²»Ê¹ÄÜ

//	AX_BEEP_On();
//	AX_Delayms(100);	
//	AX_BEEP_Off();
//	AX_Delayms(1000);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPIÉèÖÃÎªË«ÏßË«ÏòÈ«Ë«¹¤
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPIÖ÷»ú
 	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//·¢ËÍ½ÓÊÕ8Î»Ö¡½á¹¹
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//Ê±ÖÓÐü¿ÕµÍ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//Êý¾Ý²¶»ñÓÚµÚ1¸öÊ±ÖÓÑØ
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSSÐÅºÅÓÉÈí¼þ¿ØÖÆ
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;		//¶¨Òå²¨ÌØÂÊÔ¤·ÖÆµµÄÖµ:²¨ÌØÂÊÔ¤·ÖÆµÖµÎª16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//Êý¾Ý´«Êä´ÓMSBÎ»¿ªÊ¼
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCÖµ¼ÆËãµÄ¶àÏîÊ½
	SPI_Init(SPI2, &SPI_InitStructure);  //¸ù¾ÝSPI_InitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯ÍâÉèSPIx¼Ä´æÆ÷
 
	SPI_Cmd(SPI2, ENABLE); //Ê¹ÄÜSPIÍâÉè
			 
//	AX_BEEP_On();
//	AX_Delayms(100);	
//	AX_BEEP_Off();
//	AX_Delayms(1000);
	
	NRF24L01_CE=0; 			
//5	
//	AX_BEEP_On();
//	AX_Delayms(100);	
//	AX_BEEP_Off();
//	AX_Delayms(1000);
	
	NRF24L01_CSN=1;			//SPIÆ¬Ñ¡È¡Ïû  
	
//	AX_BEEP_On();
//	AX_Delayms(100);	
//	AX_BEEP_Off();
//	AX_Delayms(200);
	NRF24L01_RX_Mode();
	NRF_beep_flag=1;
	NRF_led_flag=1;
	 		 	 
}
//¼ì²â24L01ÊÇ·ñ´æÔÚ
//·µ»ØÖµ:0£¬³É¹¦;1£¬Ê§°Ü	
u8 NRF24L01_Check(void)
{
	NRF24L01_CE=0;
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_4); //spiËÙ¶ÈÎª9Mhz£¨24L01µÄ×î´óSPIÊ±ÖÓÎª10Mhz£©   	 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//Ð´Èë5¸ö×Ö½ÚµÄµØÖ·.
	NRF24L01_CE=1;
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //¶Á³öÐ´ÈëµÄµØÖ·  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//¼ì²â24L01´íÎó	
	return 0;		 //¼ì²âµ½24L01
}	 		 
//SPIÐ´¼Ä´æÆ÷
//reg:Ö¸¶¨¼Ä´æÆ÷µØÖ·
//value:Ð´ÈëµÄÖµ
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
	NRF24L01_CSN=0;                 //Ê¹ÄÜSPI´«Êä
	status =SPI2_ReadWriteByte(reg);//·¢ËÍ¼Ä´æÆ÷ºÅ 
	SPI2_ReadWriteByte(value);      //Ð´Èë¼Ä´æÆ÷µÄÖµ
	NRF24L01_CSN=1;                 //½ûÖ¹SPI´«Êä	   
	return(status);       			//·µ»Ø×´Ì¬Öµ
}
//¶ÁÈ¡SPI¼Ä´æÆ÷Öµ
//reg:Òª¶ÁµÄ¼Ä´æÆ÷
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //Ê¹ÄÜSPI´«Êä		
	SPI2_ReadWriteByte(reg);   //·¢ËÍ¼Ä´æÆ÷ºÅ
	reg_val=SPI2_ReadWriteByte(0XFF);//¶ÁÈ¡¼Ä´æÆ÷ÄÚÈÝ
	NRF24L01_CSN = 1;          //½ûÖ¹SPI´«Êä		    
	return(reg_val);           //·µ»Ø×´Ì¬Öµ
}	
//ÔÚÖ¸¶¨Î»ÖÃ¶Á³öÖ¸¶¨³¤¶ÈµÄÊý¾Ý
//reg:¼Ä´æÆ÷(Î»ÖÃ)
//*pBuf:Êý¾ÝÖ¸Õë
//len:Êý¾Ý³¤¶È
//·µ»ØÖµ,´Ë´Î¶Áµ½µÄ×´Ì¬¼Ä´æÆ÷Öµ 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           //Ê¹ÄÜSPI´«Êä
  	status=SPI2_ReadWriteByte(reg);//·¢ËÍ¼Ä´æÆ÷Öµ(Î»ÖÃ),²¢¶ÁÈ¡×´Ì¬Öµ   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI2_ReadWriteByte(0XFF);//¶Á³öÊý¾Ý
  	NRF24L01_CSN=1;       //¹Ø±ÕSPI´«Êä
  	return status;        //·µ»Ø¶Áµ½µÄ×´Ì¬Öµ
}
//ÔÚÖ¸¶¨Î»ÖÃÐ´Ö¸¶¨³¤¶ÈµÄÊý¾Ý
//reg:¼Ä´æÆ÷(Î»ÖÃ)
//*pBuf:Êý¾ÝÖ¸Õë
//len:Êý¾Ý³¤¶È
//·µ»ØÖµ,´Ë´Î¶Áµ½µÄ×´Ì¬¼Ä´æÆ÷Öµ
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //Ê¹ÄÜSPI´«Êä
  	status = SPI2_ReadWriteByte(reg);//·¢ËÍ¼Ä´æÆ÷Öµ(Î»ÖÃ),²¢¶ÁÈ¡×´Ì¬Öµ
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI2_ReadWriteByte(*pBuf++); //Ð´ÈëÊý¾Ý	 
  	NRF24L01_CSN = 1;       //¹Ø±ÕSPI´«Êä
  	return status;          //·µ»Ø¶Áµ½µÄ×´Ì¬Öµ
}				   
//Æô¶¯NRF24L01·¢ËÍÒ»´ÎÊý¾Ý
//txbuf:´ý·¢ËÍÊý¾ÝÊ×µØÖ·
//·µ»ØÖµ:·¢ËÍÍê³É×´¿ö
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
 	SPI2_SetSpeed(SPI_BaudRatePrescaler_4);//spiËÙ¶ÈÎª9Mhz£¨24L01µÄ×î´óSPIÊ±ÖÓÎª10Mhz£©   
	NRF24L01_CE=0;
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//Ð´Êý¾Ýµ½TX BUF  32¸ö×Ö½Ú
 	NRF24L01_CE=1;//Æô¶¯·¢ËÍ	   
	while(NRF24L01_IRQ!=0);//µÈ´ý·¢ËÍÍê³É
	sta=NRF24L01_Read_Reg(STATUS);  //¶ÁÈ¡×´Ì¬¼Ä´æÆ÷µÄÖµ	   
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //Çå³ýTX_DS»òMAX_RTÖÐ¶Ï±êÖ¾
	if(sta&MAX_TX)//´ïµ½×î´óÖØ·¢´ÎÊý
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//Çå³ýTX FIFO¼Ä´æÆ÷ 
		return MAX_TX; 
	}
	if(sta&TX_OK)//·¢ËÍÍê³É
	{
		return TX_OK;
	}
	return 0xff;//ÆäËûÔ­Òò·¢ËÍÊ§°Ü
}
//Æô¶¯NRF24L01½ÓÊÜÒ»´ÎÊý¾Ý
//rxbuf:´æ´¢½ÓÊÜÊý¾ÝµÄÊ×µØÖ·
//·µ»ØÖµ:0£¬½ÓÊÕÍê³É£»ÆäËû£¬´íÎó´úÂë
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
	SPI2_SetSpeed(SPI_BaudRatePrescaler_4); //spiËÙ¶ÈÎª9Mhz£¨24L01µÄ×î´óSPIÊ±ÖÓÎª10Mhz£©   
	
	sta=NRF24L01_Read_Reg(STATUS);  //¶ÁÈ¡×´Ì¬¼Ä´æÆ÷µÄÖµ    	 
	//NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //Çå³ýTX_DS»òMAX_RTÖÐ¶Ï±êÖ¾
	NRF24L01_CE=0;
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta );
	NRF24L01_CE=1;
	if(sta&RX_OK)//½ÓÊÕµ½Êý¾Ý
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//¶ÁÈ¡Êý¾Ý
		NRF24L01_CE=0;
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//Çå³ýRX FIFO¼Ä´æÆ÷ 
		NRF24L01_CE=1;
		return 0; 
	}	   
	return 1;//Ã»ÊÕµ½ÈÎºÎÊý¾Ý
}					    
//¸Ãº¯Êý³õÊ¼»¯NRF24L01µ½RXÄ£Ê½
//ÉèÖÃRXµØÖ·,Ð´RXÊý¾Ý¿í¶È,Ñ¡ÔñRFÆµµÀ,²¨ÌØÂÊºÍLNA HCURR
//µ±CE±ä¸ßºó,¼´½øÈëRXÄ£Ê½,²¢¿ÉÒÔ½ÓÊÕÊý¾ÝÁË		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE=0;	  
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//Ð´RX½ÚµãµØÖ·
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //Ê¹ÄÜÍ¨µÀ0µÄ×Ô¶¯Ó¦´ð    
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//Ê¹ÄÜÍ¨µÀ0µÄ½ÓÊÕµØÖ·  	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,0);	     //ÉèÖÃRFÍ¨ÐÅÆµÂÊ		  
	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//Ñ¡ÔñÍ¨µÀ0µÄÓÐÐ§Êý¾Ý¿í¶È 	    
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//ÉèÖÃTX·¢Éä²ÎÊý,0dbÔöÒæ,2Mbps,µÍÔëÉùÔöÒæ¿ªÆô   
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//ÅäÖÃ»ù±¾¹¤×÷Ä£Ê½µÄ²ÎÊý;PWR_UP,EN_CRC,16BIT_CRC,½ÓÊÕÄ£Ê½ 
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//Çå³ýRX FIFO¼Ä´æÆ÷ 
	NRF24L01_CE = 1; //CEÎª¸ß,½øÈë½ÓÊÕÄ£Ê½ 
	AX_Delayus(150);
}						 
//¸Ãº¯Êý³õÊ¼»¯NRF24L01µ½TXÄ£Ê½
//ÉèÖÃTXµØÖ·,Ð´TXÊý¾Ý¿í¶È,ÉèÖÃRX×Ô¶¯Ó¦´ðµÄµØÖ·,Ìî³äTX·¢ËÍÊý¾Ý,Ñ¡ÔñRFÆµµÀ,²¨ÌØÂÊºÍLNA HCURR
//PWR_UP,CRCÊ¹ÄÜ
//µ±CE±ä¸ßºó,¼´½øÈëRXÄ£Ê½,²¢¿ÉÒÔ½ÓÊÕÊý¾ÝÁË		   
//CEÎª¸ß´óÓÚ10us,ÔòÆô¶¯·¢ËÍ.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE=0;	    
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//Ð´TX½ÚµãµØÖ· 
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //ÉèÖÃTX½ÚµãµØÖ·,Ö÷ÒªÎªÁËÊ¹ÄÜACK	  

	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //Ê¹ÄÜÍ¨µÀ0µÄ×Ô¶¯Ó¦´ð    
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //Ê¹ÄÜÍ¨µÀ0µÄ½ÓÊÕµØÖ·  
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//ÉèÖÃ×Ô¶¯ÖØ·¢¼ä¸ôÊ±¼ä:500us + 86us;×î´ó×Ô¶¯ÖØ·¢´ÎÊý:10´Î
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,0);       //ÉèÖÃRFÍ¨µÀÎª0
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //ÉèÖÃTX·¢Éä²ÎÊý,0dbÔöÒæ,2Mbps,µÍÔëÉùÔöÒæ¿ªÆô   
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //ÅäÖÃ»ù±¾¹¤×÷Ä£Ê½µÄ²ÎÊý;PWR_UP,EN_CRC,16BIT_CRC,½ÓÊÕÄ£Ê½,¿ªÆôËùÓÐÖÐ¶Ï
	NRF24L01_CE=1;//CEÎª¸ß,10usºóÆô¶¯·¢ËÍ
	AX_Delayus(10);
}

void FSM_NRF_ScanKey(NRF_CTL_INFO *nrt_ctl_info)
{
		NRF24L01_RX_Mode();
    u8 sta;
    u8 tmp_buf[32];
		SPI2_SetSpeed(SPI_BaudRatePrescaler_4);
		NRF24L01_CE=1;
    // ¶ÁÈ¡×´Ì¬¼Ä´æÆ÷µÄÖµ
    sta = NRF24L01_Read_Reg(STATUS);
		
		//Display_Status_Message(sta);
		//AX_OLED_ClearScreen();
		
    // ¼ì²é RX_OK ±êÖ¾Î»

		// ½ÓÊÕµ½Êý¾Ý£¬¶ÁÈ¡Êý¾Ý°ü
//		AX_OLED_ClearScreen(); // Çå³ýÖ®Ç°µÄÏÔÊ¾ÄÚÈÝ
//		AX_OLED_DispStr(0, 0, "sta:", 0); // ÏÔÊ¾±êÇ©

//		// ÏÔÊ¾ rxbuf[0] µÄ×Ö·ûÖµ
//		AX_OLED_Disp16Char(24, 0, sta, 0); // x=24, y=0, Õý³£ÏÔÊ¾
//		NRF_beep_flag=1;
//		NRF_led_flag=1;
		//AX_Delayms(10);
		AX_OLED_ClearScreen();
		AX_OLED_DispStr(0, 0, "RX_ok:", 0);
		if (!NRF24L01_RxPacket(tmp_buf))  // Èç¹û¶ÁÈ¡³É¹¦
		{
			AX_OLED_Disp16Char(8, 0, tmp_buf[1], 0);
//			NRF_beep_flag=1;
//			NRF_led_flag=1;
			
			//AX_OLED_ClearScreen(); // Çå³ýÖ®Ç°µÄÏÔÊ¾ÄÚÈÝ
			//AX_OLED_DispStr(0, 0, "RX0:", 0); // ÏÔÊ¾±êÇ©

			// ÏÔÊ¾ rxbuf[0] µÄ×Ö·ûÖµ
			//AX_OLED_Disp16Char(24, 0, tmp_buf[1], 0); // x=24, y=0, Õý³£ÏÔÊ¾
			
			if(tmp_buf[0]==0x55 && tmp_buf[1]==0x7E &&tmp_buf[8]==0x7E&&tmp_buf[9]==0x55)
			{
					nrt_ctl_info->ST=tmp_buf[2];
					nrt_ctl_info->steering_angle=tmp_buf[3];
					//nrt_ctl_info->steering_angle_velocity=tmp_buf[4];
					nrt_ctl_info->speed=tmp_buf[5];
				NRF_beep_flag=1;
				NRF_led_flag=1;
					//nrt_ctl_info->acceleration=tmp_buf[6];
			}
			else if (tmp_buf[1]==0x55 && tmp_buf[2]==0x7E &&tmp_buf[9]==0x7E&&tmp_buf[10]==0x55)
			{
				nrt_ctl_info->ST=tmp_buf[3];
				nrt_ctl_info->steering_angle=tmp_buf[4];
				//nrt_ctl_info->steering_angle_velocity=tmp_buf[5];
				nrt_ctl_info->speed=tmp_buf[6];
				NRF_beep_flag=1;
				NRF_led_flag=1;
			}
//			if (tmp_buf[1] == 0x55 && tmp_buf[2] == 0x7E && tmp_buf[9] == 0x7E && tmp_buf[10] == 0x55)
//			{
//				// ¸üÐÂ¿ØÖÆÐÅÏ¢
//				nrt_ctl_info->ST = tmp_buf[3];
//				nrt_ctl_info->steering_angle = tmp_buf[4];
//				nrt_ctl_info->speed = tmp_buf[6];
//				NRF_beep_flag=1;
//				NRF_led_flag=1;
//				//AX_Delayms(10);
//			}
			else
			{
					// ´¦Àí´íÎóÊý¾Ý
					NRF24L01_FlushRx();  // Çå³ý RX FIFO
					//NRF_beep_flag=0;
					//NRF_led_flag=0;
			}
		}
		else
		{
			NRF24L01_FlushRx();
			NRF_beep_flag=0;
			NRF_led_flag=0;
		}

 
//		else
//		{
////			// ´¦Àí´íÎóÊý¾Ý
////			NRF24L01_FlushRx();  // Çå³ý RX FIFO
////			NRF_beep_flag=0;
////			NRF_led_flag=0;
//		}

//		// Çå³ý RX_OK ±êÖ¾
//		NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS,  status | RX_OK);

//		// Çå³ý RX FIFO£¬È·±£Ã»ÓÐ²ÐÁôÊý¾Ý
//		NRF24L01_FlushRx();
}
// 	else
// 	{
// 		// ´¦Àí´íÎóÊý¾Ý
// 		//NRF24L01_FlushRx();  // Çå³ý RX FIFO
// 		NRF_beep_flag=0;
// 		NRF_led_flag=0;
// 	}
// }

// void FSM_NRF_ScanKey(NRF_CTL_INFO *nrt_ctl_info)
// {
// 	//NRF24L01_RX_Mode();
// 	// AX_BEEP_On();
// 	// AX_Delayms(20);	 
// 	// AX_BEEP_Off();
// 	// AX_Delayms(20);	 
// 	u8 tmp_buf[32];
// 	if(!NRF24L01_RxPacket(tmp_buf))
// 	{

// 		//tmp_buf[9]=0;
// 		//for testing
// 		//AX_BEEP_On();
// 		AX_Delayus(200);
// 		NRF24L01_FlushRx();		
// 		//AX_BEEP_Off();
// 		//
		
// 		//if(tmp_buf[0]==0x55 && tmp_buf[1]==0x7E &&tmp_buf[8]==0x7E&&tmp_buf[9]==0x55&& verifyChecksum(tmp_buf)){
			
// 		if(tmp_buf[0]==0x55 && tmp_buf[1]==0x7E &&tmp_buf[8]==0x7E&&tmp_buf[9]==0x55){
		
// 		//if(tmp_buf[1]==0x55 && tmp_buf[2]==0x7E &&tmp_buf[9]==0x7E&&tmp_buf[10]==0x55){
// 			//AX_Delayms(150);
// 			nrt_ctl_info->ST=tmp_buf[2];
// 			nrt_ctl_info->steering_angle=tmp_buf[3];
// 			//nrt_ctl_info->steering_angle_velocity=tmp_buf[4];
// 			nrt_ctl_info->speed=tmp_buf[5];
// 			//nrt_ctl_info->acceleration=tmp_buf[6];
// 			//AX_BEEP_On();
// 			AX_Delayus(100);	 
// 			//AX_BEEP_Off();		
			
// 		}else{
// 			// data Error
// //			nrt_ctl_info->ST=0;
// //			nrt_ctl_info->steering_angle=0;
// //			nrt_ctl_info->steering_angle_velocity=0;
// //			nrt_ctl_info->speed=0;
// //			nrt_ctl_info->acceleration=0;
// 			//NRF24L01_FlushRx();
// 			// AX_BEEP_On();
// 			// AX_LED_Red_On();	
// 			// AX_Delayms(200); 
// 			// AX_BEEP_Off();
// 			// AX_LED_Red_Off();	
// 			// AX_BEEP_On();
// 			// AX_Delayms(20);
// 			// AX_BEEP_Off();
// 		}


// 	}else{
// 			// data Error
// //			nrt_ctl_info->ST=0;
// //			nrt_ctl_info->steering_angle=0;
// //			nrt_ctl_info->steering_angle_velocity=0;
// //			nrt_ctl_info->speed=0;
// //			nrt_ctl_info->acceleration=0;
// //			AX_Delayus(100);
		
// 	}
	

// }

/***
 * @brief Function for verifying; 1 as true, 0 as false
 * @param data
 * @return status of the verification
 */
u8 verifyChecksum(u8* data) {
  u8 checksum = 0;
  for (int i = 2; i <= 6; i++) {
    checksum ^= data[i];
  }
  if (checksum == data[7])
	return 1;
  else return 0;
}
void NRF24L01_FlushRx(void)
{
	NRF24L01_CE=0;
	NRF24L01_Write_Reg(FLUSH_RX,0xff);
	NRF24L01_CE=1;
}
void Display_Status_Message(u8 sta)
{
    // ?? OLED ??
    AX_OLED_ClearScreen();

    // ?? TX FIFO ???? (bit 0)
    if (sta & 0x01)
    {
        AX_OLED_DispStr(0, 0, (u8*)"TX FIFO FULL", 0);
    }
    else
    {
        AX_OLED_DispStr(0, 0, (u8*)"TX FIFO OK", 0);
    }

    // ????????? (bit 1-3, RX_P_NO)
    u8 rx_channel = (sta >> 1) & 0x07;  // ?? bit 1-3
    char channel_msg[16];
    //sprintf(channel_msg, "RX CH: %d", rx_channel);
    AX_OLED_DispStr(0, 2, (u8*)channel_msg, 0);

    // ???????????? (bit 4, MAX_RT)
    if (sta & 0x10)
    {
        AX_OLED_DispStr(0, 3, (u8*)"MAX RETRIES REACHED", 0);
    }
    else
    {
        AX_OLED_DispStr(0, 3, (u8*)"RETRY OK", 0);
    }

    // ???????? (bit 5, TX_DS)
    if (sta & 0x20)
    {
        AX_OLED_DispStr(0, 4, (u8*)"TX DONE", 0);
    }
    else
    {
        AX_OLED_DispStr(0, 4, (u8*)"TX NOT DONE", 0);
    }

    // ????????? (bit 6, RX_DR)
    if (sta & 0x40)
    {
        AX_OLED_DispStr(0, 5, (u8*)"RX DATA READY", 0);
    }
    else
    {
        AX_OLED_DispStr(0, 5, (u8*)"NO RX DATA", 0);
    }
}
