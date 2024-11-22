#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "printf.h"
//this is for uno
// 定义 CE 和 CSN 引脚
const int CE_PIN = 7;
const int CSN_PIN = 8;

// 定义spI通道地址
const byte RX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const byte TX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 创建 RF24 对象
RF24 radio(CE_PIN, CSN_PIN);

// 定义 ControlCommand 结构体
struct ControlCommand {
  float steering_tire_angle;
  float speed;
};
//声明 ControlCommand 结构体
ControlCommand control_command;
//声明 定长数据包
byte tx_buf[32];

void setup() {
  Serial.begin(115200);

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

  Serial.println("radio setup succeed as: ");
  radio.printPrettyDetails();
}

void loop() {

  if (Serial.available() >= 11) {
    // 读取数据
    byte data[11];
    Serial.readBytes(data, 11);

    // 验证校验和
    if (data[0] == 0x42 && verifyChecksum(data)) {
      // 解析消息
      control_command.steering_tire_angle = *(float*)(data + 2);
      control_command.speed = *(float*)(data + 6);

      set_txbuff(control_command.steering_tire_angle,control_command.speed);
      
      bool ok = radio.write(&tx_buf, sizeof(tx_buf));

      if (ok) {
        sendStateMessage(true,control_command.steering_tire_angle, control_command.speed);
      } else {
        sendStateMessage(false,control_command.steering_tire_angle, control_command.speed);
      }
    }
  }
}

bool verifyChecksum(byte* data) {
  byte checksum = 0;
  for (int i = 0; i < 10; i++) {
    checksum ^= data[i];
  }
  return checksum == data[10];
}
void sendStateMessage(bool flag, float steering_tire_angle, float speed) {
  byte data[11];
  data[0] = 0x42;
  if (flag==true)data[1] = 0x02;
  else data[1]=0x03;
  *(float*)(data + 2) = steering_tire_angle;
  *(float*)(data + 6) = speed;
  data[10] = calculateChecksumInterval(data,0,9);
  Serial.write(data, 11);
}

byte calculateChecksumInterval(byte array[], int head, int tail) {
  byte checksum = 0;
  for (int i = head; i <= tail; i++) {
    checksum ^= array[i];
  }
  return checksum;
}
void printHexArray(byte array[], int length) {
  for (int i = 0; i < length; i++) {
    Serial.print(array[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
void set_txbuff(float steering_tire_angle, float speed)
{
 
  //txbuff          0     1     2     3     4     5      6     7    8   9
  //txbuff_default 0x55  0x7e  0x01  0x00 0x00   0x00  0x00  0x?? 0x7e 0x55

  tx_buf[0] = 0x55;//包头帧1
  tx_buf[1] = 0x7E;//包头帧2
  tx_buf[2] = 0x01;//ST 0为非NRF，1为NRF
  tx_buf[3] = floatToByte(steering_tire_angle);//steering_angle,转向角度值，这里强制将float放大100倍（保留小数点后两位，后续在控制阶段还需要进行range).
  tx_buf[4] = 0x00;//steering_angle_velocity,在当前控制模式下未使用角速度控制，所以设为0.
  tx_buf[5] = floatToByte(speed);//speed,前进/后退速度值（负值为后退），这里强制将float放大100倍
  tx_buf[6] = 0x00;//acceleratio,加速度，以阶梯形式逐级提高rc小车的速度基数，每级+2.
  tx_buf[7] = calculateChecksumInterval(tx_buf,2,6);//Checksum;计算包的数据段的检验和（2-6）
  tx_buf[8] = 0x7E;//包尾帧1
  tx_buf[9] = 0x55;//包尾帧2
}
// 将 float 转换为 byte，保留两位小数精度
byte floatToByte(float value) {
  int scaledValue = (int)(value * 100);
  return (byte)scaledValue;
}

// 将 byte 转换为 float，还原两位小数精度
float byteToFloat(byte value) {
  int scaledValue = (int)value;
  return (float)scaledValue / 100.0f;
}