#include <SPI.h>
//#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

// 定义 CE 和 CSN 引脚
const int CE_PIN = 7;
const int CSN_PIN = 8;

// 定义通道地址
const byte RX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const byte TX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 创建 RF24 对象
RF24 radio(CE_PIN, CSN_PIN);

// 存储接收数据的变量
char receivedText[32] = "";
//char rx_text[5]="";
byte temp_buf[32];
byte rx_text[5];

void printHexArray(byte array[], int length) {
  for (int i = 0; i < length; i++) {
    // 打印每个字节的十六进制值，宽度为 2，不足两位补 0
    Serial.print(array[i], HEX); 
    Serial.print(" "); // 添加空格分隔每个字节
  }
  Serial.println(); // 打印换行符
}

void setup() {
  Serial.begin(9600);

  printf_begin();
  if (!radio.begin()) {
  Serial.println(F("radio hardware is not responding!"));
//while (1) {} // 停止程序
  }
  radio.printPrettyDetails();// initial info
  Serial.println("\n \n \n");
  radio.openReadingPipe(1, RX_address); //reading pipe
  radio.setPayloadSize(32);             //read_buf_size
  radio.setChannel(0);                  //2400 MHz + <channel number> 

  radio.setCRCLength(RF24_CRC_16);       //set_CRC_
  /*CRC
  RF24_CRC_DISABLED ---	to disable using CRC checksums
  RF24_CRC_8 ---	to use 8-bit checksums
  RF24_CRC_16 ---	to use 16-bit checksums
  */
  radio.setPALevel(RF24_PA_MAX);
  /**
  level (enum value)	nRF24L01
  --        |     --  
  RF24_PA_MIN (0)	    -18 dBm	
  RF24_PA_LOW (1)	    -12 dBm	
  RF24_PA_HIGH (2)	  -6  dBm	
  RF24_PA_MAX (3)	    0   dBm	
*/
  radio.setDataRate(RF24_1MBPS);
  /**
  *RF24_1MBPS   --- (0)	for 1 Mbps
  *RF24_2MBPS   --- (1)	for 2 Mbps
  *RF24_250KBPS --- (2)	for 250 kbps
  */
  radio.openWritingPipe(TX_address); 
// 设置为接收模式
  radio.startListening();
  Serial.println("the new set is ");
  radio.printPrettyDetails();// 


}

void loop() {
// 检查是否有数据接收
// radio.printPrettyDetails();
  if (radio.available()) {
// 读取数据
    radio.read(&temp_buf, sizeof(temp_buf));

    if(temp_buf[1]==0x55){
      Serial.println("temp_buf[1]--------correct");
      if(temp_buf[2]==0x7e){
        Serial.println("temp_buf[2]--------correct");
        if(temp_buf[8]==0x7e)
        Serial.println("temp_buf[8]--------correct");
        else Serial.println(temp_buf[8],HEX);
        if(temp_buf[9]==0x55)
        Serial.println("temp_buf[9]--------correct");
        else Serial.println(temp_buf[9],HEX);
      }
    }
    if(temp_buf[1]==0x55 && temp_buf[2]==0x7e && temp_buf[8]==0x7e && temp_buf[9]==0x55 )
    {
      rx_text[0]=temp_buf[3];
      rx_text[1]=temp_buf[4];
      rx_text[2]=temp_buf[5];
      rx_text[3]=temp_buf[6];
      rx_text[4]=temp_buf[7];
      printHexArray(rx_text,5);
      //Serial.write(rx_text,5);
    }else{
      Serial.println("data_analysis_error");
    }

  }
}