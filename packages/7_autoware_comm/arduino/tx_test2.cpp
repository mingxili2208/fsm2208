#include <SPI.h>
#include <RF24.h>
#include "printf.h"
//this is for uno
// 定义 CE 和 CSN 引脚
const int CE_PIN = 7;
const int CSN_PIN = 8;

// 定义通道地址
const byte RX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const byte TX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 创建 RF24 对象
RF24 radio(CE_PIN, CSN_PIN);

// 发送数据的变量
byte tx_text[5] = {0x01, 0x30, 0x00, 0x20, 0x00}; // 示例数据
byte tx_buf[32];

void printHexArray(byte array[], int length) {
  for (int i = 0; i < length; i++) {
    Serial.print(array[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  printf_begin();

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!"));
    //while (1) {}
  }
  radio.printPrettyDetails();
  Serial.println("\n \n \n");
  radio.setPayloadSize(32);
  radio.setChannel(0);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.openWritingPipe(TX_address);
  radio.stopListening(); // 停止监听，进入发送模式

  // 初始化发送数据
  tx_buf[0] = 0x55;
  tx_buf[1] = 0x7E;
  //tx_buf[2] = 0x01;
  for (int i = 0; i < 4; i++) {
    tx_buf[i + 2] = tx_text[i];
  }
  tx_buf[8] = 0x7E;
  tx_buf[9] = 0x55;

  // tx_buf[0] = 0x55;
  // tx_buf[1] = 0x7E;
  // for (int i = 0; i < 4; i++) {
  //   tx_buf[i + 2] = tx_text[i];
  // }
  // tx_buf[8] = 0x7E;
  // tx_buf[9] = 0x55;

  Serial.println("the new set is ");
  radio.printPrettyDetails();
}

void loop() {
  // 发送数据
  unsigned long startTime = micros();  // 获取开始时间
  
  // 发送数据
  bool ok = radio.write(&tx_buf, sizeof(tx_buf));

  unsigned long endTime = micros();  // 获取结束时间
  unsigned long duration = endTime - startTime;  // 计算发送耗时

  if (ok) {
    Serial.print("Data sent successfully in ");
  } else {
    Serial.print("Data sending failed in ");
  }
  Serial.print(duration);
  Serial.println(" microseconds");

  printHexArray(tx_buf, 32);

  // 添加延时，避免频繁发送
  delay(1000);

  // bool ok = radio.write(&tx_buf, sizeof(tx_buf));

  // if (ok) {
  //   Serial.println("Data sent successfully");
  //   printHexArray(tx_buf,32);
  // } else {
  //   Serial.println("Data sending failed");
  //   printHexArray(tx_buf,32);
  // }

  // // 可以添加延时，避免频繁发送
  // delay(1000);
}