#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>
//#include <queue>

#define MAX_POINTS 10

class AngleToPInterpolator {
  private:
    float angle[MAX_POINTS];
    float p[MAX_POINTS];
    int size;

  public:
    AngleToPInterpolator() : size(0) {}

    // 添加数据点
    bool addPoint(float angleVal, float pVal) {
      if (size >= MAX_POINTS) return false;
      
      // 找到正确的插入位置
      int i;
      for (i = size - 1; (i >= 0 && angle[i] > angleVal); i--) {
        angle[i + 1] = angle[i];
        p[i + 1] = p[i];
      }
      
      // 插入新点
      angle[i + 1] = angleVal;
      p[i + 1] = pVal;
      size++;
      return true;
    }

    // 执行插值，输入角度a，获得p值
    float getP(float angleVal) {
      // 边界检查
      if (size == 0) return 0;
      if (angleVal <= angle[0]) return p[0];
      if (angleVal >= angle[size - 1]) return p[size - 1];

      // 找到正确的区间
      int i = 0;
      while (i < size - 1 && angle[i + 1] < angleVal) i++;

      // 线性插值
      float a0 = angle[i], a1 = angle[i + 1];
      float p0 = p[i], p1 = p[i + 1];
      return p0 + (p1 - p0) * (angleVal - a0) / (a1 - a0);
    }
};

// 全局插值器实例
AngleToPInterpolator interp;

// 这适用于Uno
// 定义 CE 和 CSN 引脚
const int CE_PIN = 7;
const int CSN_PIN = 8;

// 定义SPI通道地址
const byte RX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 末位空置 
const byte TX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 创建 RF24 对象
RF24 radio(CE_PIN, CSN_PIN);

// 定义 ControlCommand 结构体
struct ControlCommand {
  float steering_tire_angle; // 转向角度
  float speed; // 速度
};

// 声明 ControlCommand 结构体
ControlCommand control_command;

// 声明定长数据包
byte tx_buf[32];

// 时间跟踪变量
unsigned long lastReceivedTime = 0; // 最后一次接收时间
bool timeoutSent = false; // 是否已经发送过超时消息

void setup() {
  
  Serial.begin(115200);
  
  if (!radio.begin()) {
    sendStateMessage(0x00, control_command.steering_tire_angle, control_command.speed);
    // 可以选择在此处加入错误处理逻辑，比如蜂鸣器提示或LED指示
    while (1) {} // 停止程序
  }

  radio.setPayloadSize(32); // 设置数据包大小为定长32字节（与芯片最兼容）
  radio.setChannel(0); // channel 0 as 2.4GHZ-----（发射频率=2400MHZ + n_channel）
  radio.setCRCLength(RF24_CRC_16); // 16位校验码
  radio.setPALevel(RF24_PA_MAX); // 0db
  radio.setDataRate(RF24_2MBPS); // 空中速率为2MBPS
  radio.openWritingPipe(TX_address);
  radio.stopListening(); // 停止监听，进入发送模式

  interp.addPoint(0, 0);
  interp.addPoint(2, 8);
  interp.addPoint(5, 15);
  interp.addPoint(8, 20);
  interp.addPoint(14, 50);
  interp.addPoint(15, 60);
  interp.addPoint(20, 85);
  //interp.addPoint(8, 18);
  //interp.addPoint(12, 25);
  //interp.addPoint(15, 50);
  //interp.addPoint(20, 70);
  interp.addPoint(25, 100);
  interp.addPoint(30, 120);
  // interp.addPoint(50, 150);

  // 初始化最后接收时间为当前时间
  lastReceivedTime = millis();
}

void loop() {
  // 检查是否有串口数据可读
  if (Serial.available() > 0) 
  {
    // 读取数据
    byte data[11];
    Serial.readBytes(data, 11);

    // 重置最后接收时间和超时发送标志
    lastReceivedTime = millis();
    timeoutSent = false;

    // 验证校验和
    if (data[0] == 0x42 ) {
      // 解析消息

      control_command.steering_tire_angle = *(float*)(data + 2);
      control_command.speed = *(float*)(data + 6);
      
      // data analysis succeed
      sendStateMessage(0x01, control_command.steering_tire_angle, control_command.speed);
      set_txbuff(control_command.steering_tire_angle, control_command.speed);
      
      bool ok = radio.write(&tx_buf, sizeof(tx_buf));

      if (ok)
      {
        // send succeed
        sendStateMessage(0x02, control_command.steering_tire_angle, control_command.speed);
        // 输出发送的设定值到主终端
//        Serial.print("发送命令到串口: 速度=");
//        Serial.print(control_command.speed);
//        Serial.print(" m/s, 转向角度=");
//        Serial.print(control_command.steering_tire_angle);
//        Serial.println(" 度");
      } 
      else 
      {
        // send error
        sendStateMessage(0x03, control_command.steering_tire_angle, control_command.speed);
//        Serial.println("发送命令到串口失败！");
      }
    }
    else
    {// data received but analysis error
        sendStateMessage(0x04, control_command.steering_tire_angle, control_command.speed);
//        sendStateMessage(data[1], control_command.steering_tire_angle, control_command.speed);
//        Serial.println("接收到的数据格式不正确！");
    }
  }
  else
  { 
    // 检查是否超过5秒没有接收到数据
    unsigned long currentTime = millis();
    if (((unsigned long)(currentTime - lastReceivedTime) > 5000) && !timeoutSent) {
      // 发送超时消息
      sendStateMessage(0x05, 0, 0);
//      Serial.println("5秒内未接收到数据，发送超时消息。");
      timeoutSent = true; // 设置标志，防止重复发送
    }
  }
  
  delay(20);
}

bool verifyChecksum(byte* data) {
  byte checksum = 0;
  for (int i = 0; i < 10; i++) {
    checksum ^= data[i];
  }
  return checksum == data[10];
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

// Function to calculate checksum over a range
byte calculateChecksumInterval(byte array[], int head, int tail) {
  byte checksum = 0;
  for (int i = head; i <= tail; i++) {
    checksum ^= array[i];
  }
  return checksum;
}

// 速度映射函数，应用sigmoid函数
int map_speed_sigmoid(float speed, float min_speed = 0.20, float max_speed = 1.5, float min_mapped = 35, float max_mapped = 55) {
  // 设置速度阈值
  if (speed <= 0.05) {
    return 0;
  }
  speed = min(speed, max_speed);
  // 归一化速度值
  float normalized_speed = (speed - min_speed) / (max_speed - min_speed);
  // 应用 sigmoid 函数
  float sigmoid_speed = 1.0 / (1.0 + exp(-10.0 * (normalized_speed - 0.5)));
  // 映射到目标范围
  float mapped_speed = sigmoid_speed * (max_mapped - min_mapped) + min_mapped;
  return (int)mapped_speed;
}

// 设置发送缓冲区
void set_txbuff(float steering_tire_angle, float speed)
{
  // txbuff          0     1     2     3     4     5      6     7    8   9
  // txbuff_default 0x55  0x7e  0x01  0x00 0x00   0x00  0x00  0x?? 0x7e 0x55
  tx_buf[0] = 0x55; // 包头帧1
  tx_buf[1] = 0x7E; // 包头帧2
  tx_buf[2] = 0x01; // ST 0为非NRF，1为NRF
  if(steering_tire_angle < 0)
  {
    //tx_buf[3] = (byte)( 120 - interp.getP(-steering_tire_angle*1.05));
    tx_buf[3] = (byte)( 120 - interp.getP(-steering_tire_angle));
    // Serial.println(interp.getP(-steering_tire_angle));
  }
  else{
    tx_buf[3] = (byte)( 120 + interp.getP(steering_tire_angle));
  }
  // tx_buf[4] = 0x00; // steering_angle_velocity,在当前控制模式下未使用角速度控制，所以设为0.
  
  tx_buf[4] = 0x00; // steering_angle_velocity, 在当前控制模式下未使用角速度控制，所以设为0.
  
  if(speed < 0)
  {
    tx_buf[5] = (byte)( 128 - map_speed_sigmoid(-speed));
    // Serial.println(map_speed_sigmoid(-speed));
  }
  else{
    tx_buf[5] = (byte)( 128 + map_speed_sigmoid(speed));
  }
  // tx_buf[5] = (byte) map_speed_sigmoid(speed); // speed, 前进/后退速度值（负值为后退），这里强制将float放大100倍
  tx_buf[6] = 0x00; // acceleration, 加速度，以阶梯形式逐级提高rc小车的速度基数，每级+2.
  tx_buf[7] = calculateChecksumInterval(tx_buf, 2, 6); // Checksum;计算包的数据段的检验和（2-6）
  tx_buf[8] = 0x7E; // 包尾帧1
  tx_buf[9] = 0x55; // 包尾帧2
}