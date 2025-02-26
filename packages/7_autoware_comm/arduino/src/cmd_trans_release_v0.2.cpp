#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>

// 最大数据点
#define MAX_POINTS 10

// NRF24 引脚
#define CE_PIN 7
#define CSN_PIN 8

// NRF24 地址
const byte RX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const byte TX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 角度插值器
class AngleToPInterpolator {
private:
  float angle[MAX_POINTS];
  float p[MAX_POINTS];
  int size;

public:
  AngleToPInterpolator() : size(0) {}

  bool addPoint(float angleVal, float pVal) {
    if (size >= MAX_POINTS) return false;

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
    if (size == 0) return 0;
    if (angleVal <= angle[0]) return p[0];
    if (angleVal >= angle[size - 1]) return p[size - 1];

    // 二分查找
    int low = 0, high = size - 1;
    while (low < high) {
      int mid = (low + high) / 2;
      if (angle[mid] < angleVal)
        low = mid + 1;
      else
        high = mid;
    }

    int i = low - 1;
    float a0 = angle[i], a1 = angle[i + 1];
    float p0 = p[i], p1 = p[i + 1];
    return p0 + (p1 - p0) * (angleVal - a0) / (a1 - a0);
  }
};

// NRF24 处理类
class NRF24Handler {
public:
  RF24 radio;
  byte tx_buf[32];
  byte temp_buf[32];

  uint16_t distance_2;
  uint16_t distance_3;

  NRF24Handler(int ce, int csn) : radio(ce, csn), distance_2(0), distance_3(0) {}

  void init() {
    if (!radio.begin()) {
      sendStateMessage(0x00, 0, 0);
      Serial.println(F("NRF24L01 初始化失败"));
    }

    radio.setPayloadSize(32);
    radio.setChannel(0);
    radio.setCRCLength(RF24_CRC_16);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_2MBPS);
    radio.openWritingPipe(TX_address);
    radio.openReadingPipe(1, RX_address);
    radio.startListening();
  }

  bool receiveData() {
    if (radio.available()) {
      radio.read(&temp_buf, sizeof(temp_buf));
      return true;
    }
    return false;
  }

  void sendData() {
    radio.stopListening();
    bool ok = radio.write(&tx_buf, sizeof(tx_buf));
    radio.startListening();
  }

  void parseReceivedData() {
    if (temp_buf[0] == 0x55 && temp_buf[1] == 0x7e &&
        temp_buf[8] == 0x7e && temp_buf[9] == 0x55) {
      if (temp_buf[2] == 0x03) {
        uint8_t checksum = temp_buf[3] ^ temp_buf[4] ^ temp_buf[5] ^ temp_buf[6];
        if (temp_buf[7] == checksum) { 
          distance_2 = (temp_buf[3] << 8) | temp_buf[4];
          distance_3 = (temp_buf[5] << 8) | temp_buf[6];
          sendStateMessage(0x11, (float)distance_2, (float)distance_3);
        }
      }
    }
  }
};

// 全局变量
AngleToPInterpolator interp;
NRF24Handler nrf(CE_PIN, CSN_PIN);

void setup() {
  Serial.begin(115200);
  nrf.init();

  // 添加插值点
  interp.addPoint(0, 0);
  interp.addPoint(5, 15);
  interp.addPoint(10, 30);
  interp.addPoint(15, 50);
  interp.addPoint(20, 60);
  interp.addPoint(25, 100);
  interp.addPoint(30, 120);
}

void loop() {
  if (nrf.receiveData()) {
    nrf.parseReceivedData();
  }
  delay(20);
}

// 计算校验和
byte calculateChecksumInterval(byte array[], int head, int tail) {
  byte checksum = 0;
  for (int i = head; i <= tail; i++) {
    checksum ^= array[i];
  }
  return checksum;
}

// 发送自定义协议的串口消息
void sendStateMessage(byte flag, float steering_tire_angle, float speed) {
  byte data[11];
  data[0] = 0x42;
  data[1] = flag;
  *(float *)(data + 2) = steering_tire_angle;
  *(float *)(data + 6) = speed;
  data[10] = calculateChecksumInterval(data, 0, 9);
  Serial.write(data, 11);
}

// 速度映射
int map_speed_sigmoid(float speed, float min_speed = 0.05, float max_speed = 1.8,
                      float min_mapped = 50, float max_mapped = 100) {
  if (speed <= min_speed) return 0;
  speed = min(speed, max_speed);
  float normalized_speed = (speed - min_speed) / (max_speed - min_speed);
  float sigmoid_speed = 1.0 / (1.0 + exp(-10.0 * (normalized_speed - 0.5)));
  return (int)(sigmoid_speed * (max_mapped - min_mapped) + min_mapped);
}