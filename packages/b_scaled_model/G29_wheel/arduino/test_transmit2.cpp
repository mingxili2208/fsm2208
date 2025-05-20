#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>
//#include <queue>

#define MAX_POINTS 10

// 定义时间戳相关的常量和变量
#define TIME_SYNC_HEADER 0x43
#define ACK_HEADER 0x44
#define ACK_RECEIVED 0x10
#define ACK_SENT 0x11
#define SYNC_REQUEST_TIMEOUT 10000  // 10秒超时，用于响应同步请求
#define SYNC_FAILURE_HEADER 0x45    // 同步失败消息的新标头

// 定义喇叭相关常量
#define TRUMPET_ACTIVE 0x01
#define TRUMPET_INACTIVE 0x00

// 时间戳相关变量
unsigned long localTime = 0;
bool timeIsSynced = false;
bool syncRequestReceived = false;   // 标志，用于跟踪是否收到同步请求但尚未响应
unsigned long lastSyncRequestTime = 0; // 上次同步请求的时间

// 喇叭状态变量
bool trumpetActive = false;  // 喇叭激活状态

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

int map_speed_sigmoid(float speed, float min_speed = 0.20, float max_speed = 1.3, float min_mapped = 5, float max_mapped = 40);
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

  // 初始化喇叭状态
  trumpetActive = false;

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
  localTime = millis();
}

void loop() {
  // 更新本地时间
  localTime = millis();
  
  // 检查是否有未回应的同步请求已超时
  if (syncRequestReceived && (localTime - lastSyncRequestTime > SYNC_REQUEST_TIMEOUT)) {
    // 我们未能及时响应同步请求
    sendSyncFailure(0x01); // 1 = 超时
    syncRequestReceived = false;
    sendStateMessage(0x06, 0, 0); // 同步超时的新状态码
  }
  
  // 检查是否有串口数据可读
  if (Serial.available() > 0) 
  {
    // 读取第一个字节以确定数据包类型
    byte header = Serial.peek();
    
    // 处理时间同步包
    if (header == TIME_SYNC_HEADER && Serial.available() >= 6) {
      processSyncPacket();
    }
    // 处理控制命令包
    else if (header == 0x42 && Serial.available() >= 15) {  // 增加到15字节以包含时间戳
      processControlPacket();
    }
  }
  else
  { 
    // 检查是否超过5秒没有接收到数据
    if (((unsigned long)(localTime - lastReceivedTime) > 5000) && !timeoutSent) {
      // 发送超时消息
      sendStateMessage(0x05, 0, 0);
      timeoutSent = true; // 设置标志，防止重复发送
    }
  }
  
  delay(10);
}

// 处理同步时间戳的数据包
void processSyncPacket() {
  byte data[6];
  Serial.readBytes(data, 6);
  
  if (data[0] == TIME_SYNC_HEADER) {
    // 记录收到了同步请求及其时间
    syncRequestReceived = true;
    lastSyncRequestTime = millis();
    
    // 提取PC时间戳（4字节无符号整数）
    unsigned long pcTimestamp = 0;
    pcTimestamp |= (unsigned long)data[1] << 0;
    pcTimestamp |= (unsigned long)data[2] << 8;
    pcTimestamp |= (unsigned long)data[3] << 16;
    pcTimestamp |= (unsigned long)data[4] << 24;
    
    // 设置时间同步标志
    timeIsSynced = true;
    
    // 发送ACK确认，包含PC时间和Arduino本地时间
    if (sendSyncAck(pcTimestamp)) {
      // 成功发送ack，清除请求标志
      syncRequestReceived = false;
    }
  }
}

// 发送时间同步ACK，返回成功/失败状态
bool sendSyncAck(unsigned long pcTimestamp) {
  byte response[10]; 
  response[0] = TIME_SYNC_HEADER;
  response[1] = 0x01; // 表示ACK
  
  // 原始PC时间戳（4字节）
  response[2] = (byte)(pcTimestamp & 0xFF);
  response[3] = (byte)((pcTimestamp >> 8) & 0xFF);
  response[4] = (byte)((pcTimestamp >> 16) & 0xFF);
  response[5] = (byte)((pcTimestamp >> 24) & 0xFF);
  localTime = millis();
  // 当前本地时间戳（4字节）
  response[6] = (byte)(localTime & 0xFF);
  response[7] = (byte)((localTime >> 8) & 0xFF);
  response[8] = (byte)((localTime >> 16) & 0xFF);
  response[9] = (byte)((localTime >> 24) & 0xFF);
  
  // 校验和
  byte checksum = 0;
  for (int i = 0; i < 10; i++) {
    checksum ^= response[i];
  }
  
  // 尝试发送响应
  if (Serial.availableForWrite() >= 11) {
    Serial.write(response, 10);
    Serial.write(checksum);  // 发送校验和
    return true;
  } else {
    // 缓冲区空间不足，返回失败
    return false;
  }
}

// 发送同步失败通知到PC
void sendSyncFailure(byte reason) {
  byte response[6];
  response[0] = SYNC_FAILURE_HEADER;
  response[1] = reason; // 原因代码：1 = 超时，2 = 缓冲区满

  // 当前本地时间戳（4字节）
  unsigned long current_time = millis();
  response[2] = (byte)(current_time & 0xFF);
  response[3] = (byte)((current_time >> 8) & 0xFF);
  response[4] = (byte)((current_time >> 16) & 0xFF);
  response[5] = (byte)((current_time >> 24) & 0xFF);
  
  // 校验和
  byte checksum = 0;
  for (int i = 0; i < 6; i++) {
    checksum ^= response[i];
  }
  
  Serial.write(response, 6);
  Serial.write(checksum);
}

// 处理控制命令数据包，新版本可以处理带时间戳的命令
void processControlPacket() {
  byte data[16];  // 增加长度以包含时间戳和喇叭状态位
  Serial.readBytes(data, 16);

  // 重置最后接收时间和超时发送标志
  lastReceivedTime = millis();
  timeoutSent = false;

  // 验证校验和
  if (data[0] == 0x42) {
    // 提取PC时间戳（4字节）
    unsigned long pcTimestamp = 0;
    pcTimestamp |= (unsigned long)data[2] << 0;
    pcTimestamp |= (unsigned long)data[3] << 8;
    pcTimestamp |= (unsigned long)data[4] << 16;
    pcTimestamp |= (unsigned long)data[5] << 24;
    
    // 解析控制命令
    float steering_tire_angle = *(float*)(data + 6);
    float speed = *(float*)(data + 10);
    
    // 获取喇叭状态位（位于第14字节）
    bool new_trumpet_state = (data[14] == TRUMPET_ACTIVE);
    
    // 更新喇叭状态
    trumpetActive = new_trumpet_state;
    
    // 处理转向角度映射
    byte mapped_steering;
    if (steering_tire_angle < 0) {
        mapped_steering = (byte)(120 - interp.getP(-steering_tire_angle));
    } else {
        mapped_steering = (byte)(120 + interp.getP(steering_tire_angle));
    }

    // 处理速度映射
    byte mapped_speed;
    if (speed < 0) {
        mapped_speed = (byte)(128 - map_speed_sigmoid(-speed));
    } else {
        mapped_speed = (byte)(128 + map_speed_sigmoid(speed));
    }

    // 发送接收确认ACK
    sendAckMessage(ACK_RECEIVED, pcTimestamp);
    sendDecodingStatus(0x01, steering_tire_angle, speed, mapped_steering, mapped_speed);
    // 调用 set_txbuff，传递映射后的值，并包含喇叭状态
    set_txbuff(mapped_steering, mapped_speed);

    // 发送数据
    bool ok = radio.write(&tx_buf, sizeof(tx_buf));

    if (ok) {
        // 发送成功，发送带时间戳的确认ACK
        sendAckMessage(ACK_SENT, pcTimestamp);
        
        // 发送成功状态
        sendStateMessage(0x02, (float)mapped_steering,(float)mapped_speed);
    }else {
      // send error
      sendStateMessage(0x03, control_command.steering_tire_angle, control_command.speed);
    }
  }
  else
  {
    // data received but analysis error
    sendStateMessage(0x04, control_command.steering_tire_angle, control_command.speed);
  }
}

// 发送ACK消息（直接使用完整的时间戳）
void sendAckMessage(byte ackType, unsigned long pcTimestamp) {
  byte ackData[11];
  ackData[0] = ACK_HEADER;
  ackData[1] = ackType;
  
  // PC时间戳（4字节）
  ackData[2] = (byte)(pcTimestamp & 0xFF);
  ackData[3] = (byte)((pcTimestamp >> 8) & 0xFF);
  ackData[4] = (byte)((pcTimestamp >> 16) & 0xFF);
  ackData[5] = (byte)((pcTimestamp >> 24) & 0xFF);
  unsigned long arduinoTime=millis();
  // Arduino本地时间（4字节）
  ackData[6] = (byte)(arduinoTime & 0xFF);
  ackData[7] = (byte)((arduinoTime >> 8) & 0xFF);
  ackData[8] = (byte)((arduinoTime >> 16) & 0xFF);
  ackData[9] = (byte)((arduinoTime >> 24) & 0xFF);
  
  // 校验和
  ackData[10] = 0;
  for (int i = 0; i < 10; i++) {
    ackData[10] ^= ackData[i];
  }
  
  Serial.write(ackData, 11);
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
int map_speed_sigmoid(float speed, float min_speed , float max_speed , float min_mapped , float max_mapped ) {
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

// 设置发送缓冲区，集成喇叭状态
void set_txbuff(byte mapped_steering, byte mapped_speed) {
    // txbuff          0     1     2     3     4     5      6     7    8   9
    // txbuff_default 0x55  0x7e  0x01  0x00 0x00   0x00  0x00  0x?? 0x7e 0x55
    tx_buf[0] = 0x55; // 包头帧1
    tx_buf[1] = 0x7E; // 包头帧2
    tx_buf[2] = 0x01; // ST 0为非NRF，1为NRF
    
    tx_buf[3] = mapped_steering;  // 直接使用映射后的转向角度值
    tx_buf[4] = trumpetActive ? 0x01 : 0x00;  // 喇叭状态：0x01=激活，0x00=未激活
    tx_buf[5] = mapped_speed;     // 直接使用映射后的速度值
    tx_buf[6] = 0x00;             // acceleration, 设为 0
    
    // 计算校验和（从 tx_buf[2] 到 tx_buf[6]）
    tx_buf[7] = calculateChecksumInterval(tx_buf, 2, 6);
    
    tx_buf[8] = 0x7E; // 包尾帧1
    tx_buf[9] = 0x55; // 包尾帧2
}
// Add this function to report decoding status
void sendDecodingStatus(byte flag, float orig_steering, float orig_speed, byte mapped_steering, byte mapped_speed) {
  byte data[16];
  data[0] = 0x46; // New header for decoding status
  data[1] = flag; // Status flag
  
  // Original values
  *(float*)(data + 2) = orig_steering;
  *(float*)(data + 6) = orig_speed;
  
  // Mapped values
  data[10] = mapped_steering;
  data[11] = mapped_speed;
  
  // Trumpet state
  data[12] = trumpetActive ? 0x01 : 0x00;
  
  // Reserved byte
  data[13] = 0x00;
  
  // Calculate checksum over bytes 1-13
  byte checksum = 0;
  for (int i = 1; i <= 13; i++) {
    checksum ^= data[i];
  }
  data[14] = checksum;
  data[15] = 0xAA; // End marker
  
  Serial.write(data, 16);
}