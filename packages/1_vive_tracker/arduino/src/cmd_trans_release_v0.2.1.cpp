#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>
#include "printf.h"

#define MAX_POINTS 10

class AngleToPInterpolator {
private:
  float angle[MAX_POINTS];
  float p[MAX_POINTS];
  int size;

public:
  AngleToPInterpolator() : size(0) {}

  bool addPoint(float angleVal, float pVal) {
    if (size >= MAX_POINTS)
      return false;

    int i;
    for (i = size - 1; (i >= 0 && angle[i] > angleVal); i--) {
      angle[i + 1] = angle[i];
      p[i + 1] = p[i];
    }

    angle[i + 1] = angleVal;
    p[i + 1] = pVal;
    size++;
    return true;
  }

  float getP(float angleVal) {
    if (size == 0)
      return 0;
    if (angleVal <= angle[0])
      return p[0];
    if (angleVal >= angle[size - 1])
      return p[size - 1];

    int i = 0;
    while (i < size - 1 && angle[i + 1] < angleVal)
      i++;

    float a0 = angle[i], a1 = angle[i + 1];
    float p0 = p[i], p1 = p[i + 1];
    return p0 + (p1 - p0) * (angleVal - a0) / (a1 - a0);
  }
};

// 全局插值器实例
AngleToPInterpolator interp;

// 定义 CE 和 CSN 引脚
const int CE_PIN = 7;
const int CSN_PIN = 8;

// 定义 SPI 通道地址
const byte RX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
const byte TX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 创建 RF24 对象
RF24 radio(CE_PIN, CSN_PIN);

// 定义 ControlCommand 结构体
struct ControlCommand {
  float steering_tire_angle;
  float speed;
};

ControlCommand control_command;

byte tx_buf[32];  // 发送缓冲区
byte temp_buf[32];  // 接收缓冲区
uint16_t distance_2;
uint16_t distance_3;

unsigned long lastReceivedTime = 0;
bool timeoutSent = false;

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    sendStateMessage(0x00, control_command.steering_tire_angle, control_command.speed);
    while (1);
  }

  radio.setPayloadSize(32);
  radio.setChannel(0);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.openWritingPipe(TX_address);
  radio.openReadingPipe(1, RX_address);
  radio.startListening();

  interp.addPoint(0, 0);
  interp.addPoint(5, 15);
  interp.addPoint(10, 30);
  interp.addPoint(15, 50);
  interp.addPoint(20, 60);
  interp.addPoint(25, 100);
  interp.addPoint(30, 120);

  lastReceivedTime = millis();
}

void loop() {
  if (radio.available()) {
    radio.read(&temp_buf, sizeof(temp_buf));

    if (temp_buf[0] == 0x55 && temp_buf[1] == 0x7e && temp_buf[8] == 0x7e && temp_buf[9] == 0x55) {
      if (temp_buf[2] == 0x03) { // 接收位姿数据
        uint8_t checksum = temp_buf[3] ^ temp_buf[4] ^ temp_buf[5] ^ temp_buf[6];
        if (temp_buf[7] == checksum) {
          distance_2 = (temp_buf[3] << 8) | temp_buf[4];
          distance_3 = (temp_buf[5] << 8) | temp_buf[6];
          sendStateMessage(0x11, (float)distance_2, (float)distance_3);
        }
      }
    }
  }

  if (Serial.available() > 0) {
    byte data[11];
    Serial.readBytes(data, 11);

    lastReceivedTime = millis();
    timeoutSent = false;

    if (data[0] == 0x42) {
      if (data[1] == 0x01) {  // 0x01: 解析PC发送的控制指令
        control_command.steering_tire_angle = *(float *)(data + 2);
        control_command.speed = *(float *)(data + 6);

        sendStateMessage(0x01, control_command.steering_tire_angle, control_command.speed);
        set_txbuff(0x01, control_command.steering_tire_angle, control_command.speed);

        radio.stopListening();
        bool ok = radio.write(&tx_buf, sizeof(tx_buf));
        radio.startListening();

        if (ok) {
          sendStateMessage(0x02, control_command.steering_tire_angle, control_command.speed);
        } else {
          sendStateMessage(0x03, control_command.steering_tire_angle, control_command.speed);
        }
      } 
      else if (data[1] == 0x02) {  // 0x02: 请求位姿数据
        set_txbuff(0x02,0,0);
        radio.stopListening();
        bool ok = radio.write(&tx_buf, sizeof(tx_buf));
        radio.startListening();

        if (ok) {
          sendStateMessage(0x12, 0, 0); // 发送请求成功
        } else {
          sendStateMessage(0x13, 0, 0); // 发送请求失败
        }
      }
    }
  } else {
    unsigned long currentTime = millis();
    if ((currentTime - lastReceivedTime > 5000) && !timeoutSent) {
      sendStateMessage(0x05, 0, 0);
      timeoutSent = true;
    }
  }

  delay(20);
}

// **速度映射函数**
int map_speed_sigmoid(float speed, float min_speed = 0.05, float max_speed = 1.8,
                      float min_mapped = 50, float max_mapped = 100) {
  if (speed <= 0.05) {
    return 0;
  }
  speed = min(speed, max_speed);
  float normalized_speed = (speed - min_speed) / (max_speed - min_speed);
  float sigmoid_speed = 1.0 / (1.0 + exp(-10.0 * (normalized_speed - 0.5)));
  float mapped_speed = sigmoid_speed * (max_mapped - min_mapped) + min_mapped;
  return (int)mapped_speed;
}

// **设置发送缓冲区**
void set_txbuff(byte flag, float steering_tire_angle, float speed) {
  tx_buf[0] = 0x55;
  tx_buf[1] = 0x7E;
  tx_buf[2] = flag;

  tx_buf[3] = (byte)(120 + (steering_tire_angle < 0 ? -interp.getP(-steering_tire_angle) : interp.getP(steering_tire_angle)));
  tx_buf[4] = 0x00;
  tx_buf[5] = (byte)(128 + (speed < 0 ? -map_speed_sigmoid(-speed) : map_speed_sigmoid(speed)));
  tx_buf[6] = 0x00;
  tx_buf[7] = tx_buf[2] ^ tx_buf[3] ^ tx_buf[4] ^ tx_buf[5] ^ tx_buf[6];
  tx_buf[8] = 0x7E;
  tx_buf[9] = 0x55;
}
void sendStateMessage(byte flag, float steering_tire_angle, float speed) {
  byte data[11];
  data[0] = 0x42;
  data[1] = flag;
  *(float*)(data + 2) = steering_tire_angle;
  *(float*)(data + 6) = speed;
  data[10] = calculateChecksumInterval(data, 1, 9);
  Serial.write(data, 11);
}