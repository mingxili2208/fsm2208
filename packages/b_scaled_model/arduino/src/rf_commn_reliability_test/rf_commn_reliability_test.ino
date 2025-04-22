#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <printf.h>

// 常量定义
#define FRAME_HEADER1   0xAA  // 主帧头标识
#define FRAME_HEADER2   0x7E  // 备用帧头标识
#define FRAME_TAIL      0x55  // 帧尾标识

// 数据包类型定义
enum PacketType {
  PACKET_MOTION = 0x01,   // 运动数据包（包含线速度、角度等）
  PACKET_COMMAND = 0x02,  // 命令数据包
  PACKET_STATUS = 0x03,   // 状态数据包
  PACKET_ACK = 0x04       // 确认数据包
};

// 有效数据包结构（15字节有效数据 - 双帧头）
struct MotionData {
  uint8_t header1;           // 主帧头 (1字节)
  uint8_t header2;           // 备用帧头 (1字节)
  uint8_t type;              // 包类型 (1字节)
  uint8_t deviceId;          // 设备ID (1字节)
  
  int16_t linearVelocity;    // X方向线速度 (2字节)
  int16_t angularVelocity;   // Z方向角速度 (2字节)
  int16_t linearAccel;       // 线加速度 (2字节)
  int16_t angularAccel;      // 角加速度 (2字节)
  
  uint8_t sequence;          // 序列号 (1字节)
  uint8_t checksum;          // 校验和 (1字节)
  uint8_t tail;              // 帧尾 (1字节)
};  // 总计15字节有效数据

// 全局可修改配置
// RF24地址配置（5字节地址）
const uint64_t RF24_ADDRESS = 0xF0F0F0F066LL;  // 通信地址，发送和接收需使用相同地址

// RF24通道配置
// channel范围: 0-125 (对应2.4GHz - 2.525GHz)
// 推荐使用通道115-125，这个范围通常高于WiFi信号，减少干扰
const uint8_t RF24_CHANNEL = 115;  

// RF24数据速率配置
// RF24_250KBPS: 250kbps，最长距离，但传输速率最低
// RF24_1MBPS: 1Mbps，中等距离和速率的平衡选择
// RF24_2MBPS: 2Mbps，最高速率，但传输距离最短
const rf24_datarate_e RF24_DATA_RATE = RF24_1MBPS;

// RF24发射功率配置
// RF24_PA_MIN: -18dBm, 最小功率，适合近距离通信
// RF24_PA_LOW: -12dBm, 低功率
// RF24_PA_HIGH: -6dBm, 高功率
// RF24_PA_MAX: 0dBm, 最大功率，适合长距离通信
const rf24_pa_dbm_e RF24_POWER_LEVEL = RF24_PA_MAX;

// 帧头状态枚举类型
enum FrameHeaderStatus {
  HEADERS_NONE = 0,      // 无帧头
  HEADER1_ONLY = 1,      // 只有主帧头
  HEADER2_ONLY = 2,      // 只有备用帧头
  HEADERS_BOTH = 3       // 两个帧头都存在
};

// 数据跟踪统计结构体
struct TransmissionStats {
  unsigned long totalPacketsReceived;
  unsigned long goodPackets;
  unsigned long badPackets;
  unsigned long lastSequence;
  unsigned long missedSequences;
  unsigned long duplicateSequences;
  unsigned long outOfOrderSequences;
  unsigned long lastPacketTime;
  unsigned long maxInterPacketTime;
  unsigned long totalInterPacketTime;
  unsigned long packetTimeCount;
  unsigned long testStartTime;
  bool testRunning;
};

// RF24通信类封装
class RF24Manager {
private:
  RF24 radio;
  byte buffer[32];
  TransmissionStats stats;
  
public:
  // 构造函数
  RF24Manager(uint8_t cePin, uint8_t csnPin) : radio(cePin, csnPin) {
    // 初始化统计数据
    resetStats();
  }
  
  // 重置统计数据
  void resetStats() {
    stats.totalPacketsReceived = 0;
    stats.goodPackets = 0;
    stats.badPackets = 0;
    stats.lastSequence = 255; // 初始化为255，这样第一个0会被正确识别为连续
    stats.missedSequences = 0;
    stats.duplicateSequences = 0;
    stats.outOfOrderSequences = 0;
    stats.lastPacketTime = 0;
    stats.maxInterPacketTime = 0;
    stats.totalInterPacketTime = 0;
    stats.packetTimeCount = 0;
    stats.testStartTime = millis();
    stats.testRunning = false;
  }
  
  // 开始测试
  void startTest() {
    resetStats();
    stats.testRunning = true;
    stats.testStartTime = millis();
    Serial.println(F("开始传输测试..."));
  }
  
  // 结束测试并打印结果
  void endTest() {
    if (!stats.testRunning) return;
    
    unsigned long duration = millis() - stats.testStartTime;
    float durationSeconds = duration / 1000.0;
    
    Serial.println(F("\n===== 传输测试结果 ====="));
    Serial.print(F("测试持续时间: ")); Serial.print(durationSeconds); Serial.println(F(" 秒"));
    Serial.print(F("总接收包数: ")); Serial.println(stats.totalPacketsReceived);
    Serial.print(F("有效包数: ")); Serial.println(stats.goodPackets);
    Serial.print(F("无效包数: ")); Serial.println(stats.badPackets);
    
    // 计算每秒数据包数
    float packetsPerSecond = stats.totalPacketsReceived / durationSeconds;
    Serial.print(F("平均接收速率: ")); Serial.print(packetsPerSecond, 2); Serial.println(F(" 包/秒"));
    Serial.print(F("数据比特率: ")); Serial.print(packetsPerSecond * 15 * 8, 2); Serial.println(F(" 比特/秒"));
    
    // 计算丢包率
    if (stats.goodPackets > 0) {
      float packetLossRate = (float)stats.missedSequences / (stats.goodPackets + stats.missedSequences) * 100.0;
      Serial.print(F("估计丢包率: ")); Serial.print(packetLossRate, 2); Serial.println(F("%"));
      Serial.print(F("重复包数: ")); Serial.println(stats.duplicateSequences);
      Serial.print(F("乱序包数: ")); Serial.println(stats.outOfOrderSequences);
    }
    
    // 包间隔时间
    if (stats.packetTimeCount > 0) {
      float avgInterPacketTime = (float)stats.totalInterPacketTime / stats.packetTimeCount;
      Serial.print(F("平均包间隔: ")); Serial.print(avgInterPacketTime); Serial.println(F(" 毫秒"));
      Serial.print(F("最大包间隔: ")); Serial.print(stats.maxInterPacketTime); Serial.println(F(" 毫秒"));
    }
    
    stats.testRunning = false;
  }
  
  // 更新序列号统计
  void updateSequenceStats(uint8_t sequence) {
    if (stats.lastSequence == 255) {
      // 首次接收，仅记录
      stats.lastSequence = sequence;
      return;
    }
    
    // 计算期望的下一个序列号
    uint8_t expectedSequence = (stats.lastSequence + 1) & 0xFF;
    
    if (sequence == stats.lastSequence) {
      // 重复的序列号
      stats.duplicateSequences++;
    } else if (sequence == expectedSequence) {
      // 正常的连续序列
    } else {
      // 不连续序列
      stats.outOfOrderSequences++;
      
      // 计算丢失的包数 (考虑环绕)
      int missed;
      if (sequence > stats.lastSequence) {
        missed = sequence - stats.lastSequence - 1;
      } else {
        // 处理环绕情况 (0, 1, 2, ..., 254, 255, 0, 1, ...)
        missed = 256 - stats.lastSequence - 1 + sequence;
      }
      
      if (missed > 0) {
        stats.missedSequences += missed;
        Serial.print(F("丢失序列号: "));
        Serial.print(stats.lastSequence);
        Serial.print(F(" 到 "));
        Serial.print(sequence);
        Serial.print(F(" (丢失 "));
        Serial.print(missed);
        Serial.println(F(" 个包)"));
      }
    }
    
    // 更新上一次接收的序列号
    stats.lastSequence = sequence;
  }
  
  // 更新时间统计
  void updateTimeStats() {
    unsigned long currentTime = millis();
    
    if (stats.lastPacketTime > 0) {
      unsigned long interPacketTime = currentTime - stats.lastPacketTime;
      stats.totalInterPacketTime += interPacketTime;
      stats.packetTimeCount++;
      
      if (interPacketTime > stats.maxInterPacketTime) {
        stats.maxInterPacketTime = interPacketTime;
      }
    }
    
    stats.lastPacketTime = currentTime;
  }
  
  // 初始化RF24模块
  bool initialize(uint8_t channel = RF24_CHANNEL, 
                 rf24_pa_dbm_e powerLevel = RF24_POWER_LEVEL, 
                 rf24_datarate_e dataRate = RF24_DATA_RATE) {
    bool success = radio.begin();
    if (success) {
      radio.setAddressWidth(5);
      radio.setChannel(channel);  
      radio.setPALevel(powerLevel); 
      radio.setDataRate(dataRate);
      // 开启自动应答以提高可靠性
      radio.setAutoAck(true);
      // 设置较短的重试等待时间和较多的重试次数
      radio.setRetries(2, 15);
    }
    return success;
  }
  
  // 配置为接收模式
  void setupReceiver(uint64_t address = RF24_ADDRESS, uint8_t pipe = 1) {
    radio.openReadingPipe(pipe, address);
    radio.startListening();
  }
  
  // 配置为发送模式
  void setupTransmitter(uint64_t address = RF24_ADDRESS) {
    radio.openWritingPipe(address);
    radio.stopListening();
  }
  
  // 读取数据
  bool receiveData(byte* data, uint8_t* size = nullptr) {
    if (!radio.available()) {
      return false;
    }
    
    uint8_t bytes = radio.getPayloadSize();
    radio.read(buffer, bytes);
    
    // 更新收到的数据包计数
    if (stats.testRunning) {
      stats.totalPacketsReceived++;
      updateTimeStats();
    }
    
    // 复制数据到输出缓冲区
    memcpy(data, buffer, bytes);
    
    if (size != nullptr) {
      *size = bytes;
    }
    
    return true;
  }
  
  // 发送数据
  bool sendData(byte* data, uint8_t size) {
    return radio.write(data, size);
  }
  
  // 发送ACK包
  bool sendAck(uint8_t sequence) {
    byte ackPacket[4];
    ackPacket[0] = FRAME_HEADER1;
    ackPacket[1] = FRAME_HEADER2;
    ackPacket[2] = PACKET_ACK;
    ackPacket[3] = sequence;
    
    // 临时切换到发送模式
    radio.stopListening();
    bool success = radio.write(ackPacket, 4);
    radio.startListening(); // 切回接收模式
    
    return success;
  }
  
  // 打印RF24详细信息
  void printDetails() {
    radio.printDetails();
  }
  
  // 获取底层RF24对象（如果需要直接访问）
  RF24* getRF24() {
    return &radio;
  }
  
  // 获取统计信息
  TransmissionStats getStats() {
    return stats;
  }
  
  // 标记良好包
  void markGoodPacket(uint8_t sequence) {
    if (stats.testRunning) {
      stats.goodPackets++;
      updateSequenceStats(sequence);
    }
  }
  
  // 标记错误包
  void markBadPacket() {
    if (stats.testRunning) {
      stats.badPackets++;
    }
  }
};

// 内联查找帧尾的辅助函数，提高运行速度
inline bool findFrameTail(const uint8_t* data, int startPos, int maxLength, int* tailPos) {
  for (int i = startPos; i < maxLength; i++) {
    if (data[i] == FRAME_TAIL) {
      *tailPos = i;
      return true;
    }
  }
  return false;
}

// 数据处理类 - 用于处理和解析接收到的数据
class DataProcessor {
private:
  // 优化的校验和计算 - 现在正确考虑帧头偏移量
  uint8_t calculateChecksum(const uint8_t* data, FrameHeaderStatus headerStatus, uint8_t dataOffset, size_t length) {
    uint8_t sum = 0;
    
    // 校验和计算应该从包类型字段开始，不包括帧头
    // dataOffset指向数据包的包类型字段
    for (size_t i = dataOffset; i < length; i++) {
      sum += data[i];
    }
    
    return sum;
  }
  
public:
  // 检测帧头状态
  FrameHeaderStatus detectHeaderStatus(const uint8_t* data) {
    bool hasHeader1 = (data[0] == FRAME_HEADER1);
    bool hasHeader2 = false;
    
    if (hasHeader1) {
      // 如果有主帧头，检查下一个字节是否为备用帧头
      hasHeader2 = (data[1] == FRAME_HEADER2);
    } else {
      // 如果没有主帧头，检查第一个字节是否为备用帧头
      hasHeader2 = (data[0] == FRAME_HEADER2);
    }
    
    if (hasHeader1 && hasHeader2) return HEADERS_BOTH;
    if (hasHeader1) return HEADER1_ONLY;
    if (hasHeader2) return HEADER2_ONLY;
    return HEADERS_NONE;
  }
  
  // 验证MotionData数据包
  bool validateMotionData(const uint8_t* data, FrameHeaderStatus headerStatus, uint8_t& dataOffset, int dataLength) {
    // 根据帧头状态设置数据偏移量
    int tailPosition = -1;
    int checksumPosition = -1;
    
    // 设置起始偏移量
    switch (headerStatus) {
      case HEADERS_BOTH:
        dataOffset = 2;  // 两个帧头，数据从第3个字节开始
        break;
        
      case HEADER1_ONLY:
      case HEADER2_ONLY:
        dataOffset = 1;  // 单帧头，数据从第2个字节开始
        break;
        
      case HEADERS_NONE:
      default:
        Serial.println(F("错误: 未检测到有效的帧头"));
        return false;
    }
    
    // 查找第一个帧尾标识
    bool foundTail = findFrameTail(data, dataOffset, dataLength, &tailPosition);
    if (!foundTail) {
      Serial.println(F("错误: 未找到帧尾标识"));
      return false;
    }
    
    // 帧尾前一个字节是校验和
    checksumPosition = tailPosition - 1;
    if (checksumPosition <= dataOffset) {
      Serial.println(F("错误: 数据结构错误，无足够空间存放有效数据"));
      return false;
    }
    
    // 验证帧尾
    if (data[tailPosition] != FRAME_TAIL) {
      Serial.print(F("帧尾错误: 位置="));
      Serial.print(tailPosition);
      Serial.print(F(", 值=0x"));
      Serial.println(data[tailPosition], HEX);
      return false;
    }
    
    // 验证校验和 (不包括校验和字段和帧尾)
    uint8_t expectedChecksum = data[checksumPosition];
    uint8_t calcChecksum = calculateChecksum(data, headerStatus, dataOffset, checksumPosition);
    
    if (calcChecksum != expectedChecksum) {
      Serial.print(F("校验和错误: 计算值=0x"));
      Serial.print(calcChecksum, HEX);
      Serial.print(F(", 包中值=0x"));
      Serial.println(expectedChecksum, HEX);
      return false;
    }
    
    return true;
  }
  
  // 解析运动数据包
  bool unpackMotionData(const uint8_t* data, int dataLength,
                       uint8_t& deviceId,
                       int16_t& linearVel, int16_t& angularVel,
                       int16_t& linearAcc, int16_t& angularAcc,
                       uint8_t& sequence) {
    
    // 首先检测帧头状态
    FrameHeaderStatus headerStatus = detectHeaderStatus(data);
    
    // 如果没有帧头，直接返回失败
    if (headerStatus == HEADERS_NONE) {
      Serial.println(F("无法解析: 未检测到有效帧头"));
      return false;
    }
    
    // 验证数据包并获取数据偏移量
    uint8_t dataOffset;
    if (!validateMotionData(data, headerStatus, dataOffset, dataLength)) {
      return false;
    }
    
    // 确保是运动数据包
    if (data[dataOffset] != PACKET_MOTION) {
      Serial.print(F("非运动数据包类型: 0x"));
      Serial.println(data[dataOffset], HEX);
      return false;
    }
    
    // 计算各字段偏移量
    int deviceIdPos = dataOffset + 1;
    
    // 确保索引不会越界
    if (deviceIdPos >= dataLength) {
      Serial.println(F("错误: 数据包过短，无法解析设备ID"));
      return false;
    }
    
    // 提取设备ID
    deviceId = data[deviceIdPos];
    
    // 计算速度和加速度字段的偏移量
    int linearVelPos = deviceIdPos + 1;
    int angularVelPos = linearVelPos + 2;
    int linearAccPos = angularVelPos + 2;
    int angularAccPos = linearAccPos + 2;
    int sequencePos = angularAccPos + 2;
    
    // 确保索引不会越界
    if (sequencePos >= dataLength) {
      Serial.println(F("错误: 数据包过短，无法解析完整数据"));
      return false;
    }
    
    // 解析2字节整数值 (使用小端序) - 优化读取方式
    linearVel = (int16_t)(data[linearVelPos] | (data[linearVelPos+1] << 8));
    angularVel = (int16_t)(data[angularVelPos] | (data[angularVelPos+1] << 8));
    linearAcc = (int16_t)(data[linearAccPos] | (data[linearAccPos+1] << 8));
    angularAcc = (int16_t)(data[angularAccPos] | (data[angularAccPos+1] << 8));
    
    sequence = data[sequencePos];
    
    return true;
  }
  
  // 打印数据包内容（调试用）
  void printPacketDetails(const uint8_t* data, int length) {
    Serial.println(F("详细数据包内容:"));
    for (int i = 0; i < length; i++) {
      Serial.print(F("索引 "));
      Serial.print(i);
      Serial.print(F(": 0x"));
      if (data[i] < 16) Serial.print(F("0"));
      Serial.println(data[i], HEX);
    }
  }
};

// 定义命令控制变量
enum OperationMode {
  MODE_NORMAL,
  MODE_DATA_RATE_TEST,
  MODE_PACKET_LOSS_TEST
};

// 创建RF24Manager实例 (CE引脚=7, CSN引脚=8)
RF24Manager rfManager(7, 8);
DataProcessor dataProcessor;
byte receivedData[32];
OperationMode currentMode = MODE_NORMAL;
unsigned long testStartTime = 0;
unsigned long testDuration = 10000; // 默认测试持续10秒

void setup() {
  Serial.begin(115200);
  printf_begin();
  Serial.println(F("RF-NANO v4.0 运动数据接收测试 - 带数据传输率和丢包率检测"));
  Serial.println(F("---------------------------------------------------"));

  // 初始化RF模块 - 使用全局配置的默认参数
  if (rfManager.initialize()) {
    // 设置为接收模式 - 使用全局配置的默认地址
    rfManager.setupReceiver();
    Serial.println(F("RF24接收器初始化成功"));
    rfManager.printDetails();
  } else {
    Serial.println(F("RF24初始化失败"));
  }
  
  Serial.println(F("等待接收数据..."));
  Serial.println(F("命令："));
  Serial.println(F("  'r'[时间(秒)] - 开始数据率测试, 例如 'r10' 测试10秒"));
  Serial.println(F("  'p'[包数]     - 开始丢包率测试, 例如 'p100' 测试100个包"));
  Serial.println(F("  'e'          - 结束当前测试"));
  Serial.println(F("  's'          - 显示当前统计数据"));
  Serial.println();
}

void processSerialCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'r': // 开始数据率测试
      case 'R':
        {
          // 读取后续数字作为测试时间
          int testTime = 10; // 默认10秒
          delay(10); // 等待参数到达
          if (Serial.available()) {
            testTime = Serial.parseInt();
          }
          if (testTime <= 0) testTime = 10;
          
          testDuration = testTime * 1000; // 转换为毫秒
          currentMode = MODE_DATA_RATE_TEST;
          rfManager.startTest();
          testStartTime = millis();
          
          Serial.print(F("开始数据率测试，持续 "));
          Serial.print(testTime);
          Serial.println(F(" 秒"));
        }
        break;
        
      case 'p': // 开始丢包率测试
      case 'P':
        {
          // 读取后续数字作为测试包数
          int packetCount = 100; // 默认100个包
          delay(10); // 等待参数到达
          if (Serial.available()) {
            packetCount = Serial.parseInt();
          }
          if (packetCount <= 0) packetCount = 100;
          
          currentMode = MODE_PACKET_LOSS_TEST;
          rfManager.startTest();
          
          Serial.print(F("开始丢包率测试，期待 "));
          Serial.print(packetCount);
          Serial.println(F(" 个包"));
        }
        break;
        
      case 'e': // 结束当前测试
      case 'E':
        if (currentMode != MODE_NORMAL) {
          rfManager.endTest();
          currentMode = MODE_NORMAL;
          Serial.println(F("测试已手动结束"));
        }
        break;
        
      case 's': // 显示当前统计
      case 'S':
        {
          TransmissionStats stats = rfManager.getStats();
          Serial.println(F("\n当前统计信息:"));
          Serial.print(F("接收包总数: ")); Serial.println(stats.totalPacketsReceived);
          Serial.print(F("有效包数: ")); Serial.println(stats.goodPackets);
          Serial.print(F("无效包数: ")); Serial.println(stats.badPackets);
          Serial.print(F("丢失序列数: ")); Serial.println(stats.missedSequences);
          Serial.print(F("重复序列数: ")); Serial.println(stats.duplicateSequences);
          Serial.print(F("乱序包数: ")); Serial.println(stats.outOfOrderSequences);
          
          if (stats.packetTimeCount > 0) {
            float avgTime = (float)stats.totalInterPacketTime / stats.packetTimeCount;
            Serial.print(F("平均包间隔: ")); Serial.print(avgTime); Serial.println(F(" 毫秒"));
            Serial.print(F("估计数据率: ")); Serial.print(1000.0 / avgTime); Serial.println(F(" 包/秒"));
            Serial.print(F("估计比特率: ")); Serial.print((1000.0 / avgTime) * 15 * 8); Serial.println(F(" 比特/秒"));
          }
        }
        break;
    }
    
    // 清空剩余的输入缓冲区
    while (Serial.available()) {
      Serial.read();
    }
  }
}

void loop() {
  // 处理串口命令
  processSerialCommand();
  
  // 检查当前测试模式是否需要结束
  if (currentMode == MODE_DATA_RATE_TEST) {
    if (millis() - testStartTime >= testDuration) {
      rfManager.endTest();
      currentMode = MODE_NORMAL;
      Serial.println(F("数据率测试已完成"));
    }
  }
  
  // 读取数据
  uint8_t size;
  if (rfManager.receiveData(receivedData, &size)) {
    // 打印接收到的原始数据
    if (currentMode == MODE_NORMAL) {
      Serial.print(F("\n==== 接收到数据 (大小="));
      Serial.print(size);
      Serial.println(F(") ===="));
      
      // 以十六进制打印完整数据
      Serial.print(F("十六进制数据: "));
      for (int i = 0; i < min(size, 24); i++) { // 限制打印长度提高效率
        if (receivedData[i] < 16) Serial.print(F("0"));
        Serial.print(receivedData[i], HEX);
        Serial.print(F(" "));
        if ((i + 1) % 8 == 0) Serial.print(F(" ")); // 空格分隔，不用换行提高效率
      }
      Serial.println();
    }
    
    // 直接检查从索引1开始的数据 - 不关心buffer[0]
    uint8_t* actualData = &receivedData[1];
    int actualDataLength = size - 1;  // 实际数据长度（不包括buffer[0]）
    
    // 从数据处理器中获取帧头状态
    FrameHeaderStatus headerStatus = dataProcessor.detectHeaderStatus(actualData);
    
    // 根据帧头状态进行处理
    if (headerStatus == HEADERS_NONE) {
      rfManager.markBadPacket();
      if (currentMode == MODE_NORMAL) {
        Serial.println(F("未检测到任何帧头，数据无效"));
      }
      return;  // 无帧头，不处理数据
    }
    
    // 有帧头的情况下继续解析数据
    uint8_t deviceId;
    int16_t linearVel, angularVel;
    int16_t linearAcc, angularAcc;
    uint8_t sequence;
    
    // 传递从receivedData[1]开始的数据给解析函数
    if (dataProcessor.unpackMotionData(actualData, actualDataLength,
                                     deviceId, linearVel, angularVel,
                                     linearAcc, angularAcc, sequence)) {
      // 解析成功，标记为好的数据包
      rfManager.markGoodPacket(sequence);
      
      // 发送ACK包
      rfManager.sendAck(sequence);
      
      // 在常规模式下输出解析结果
      if (currentMode == MODE_NORMAL) {
        // 输出帧头类型
        switch (headerStatus) {
          case HEADERS_BOTH:
            Serial.println(F("检测到双帧头 (0xAA, 0x7E)"));
            break;
          case HEADER1_ONLY:
            Serial.println(F("仅检测到主帧头 (0xAA)"));
            break;
          case HEADER2_ONLY:
            Serial.println(F("仅检测到备用帧头 (0x7E)"));
            break;
        }
        
        Serial.println(F("解析成功，运动数据:"));
        Serial.print(F("设备ID: ")); Serial.println(deviceId);
        Serial.print(F("线速度: ")); Serial.print(linearVel); Serial.println(F(" (1/1000 m/s)"));
        Serial.print(F("角速度: ")); Serial.print(angularVel); Serial.println(F(" (1/1000 rad/s)"));
        Serial.print(F("线加速度: ")); Serial.print(linearAcc); Serial.println(F(" (1/1000 m/s²)"));
        Serial.print(F("角加速度: ")); Serial.print(angularAcc); Serial.println(F(" (1/1000 rad/s²)"));
        Serial.print(F("序列号: ")); Serial.println(sequence);
        
        // 计算实际物理值
        float realLinearVel = linearVel / 1000.0;
        float realAngularVel = angularVel / 1000.0;
        
        Serial.print(F("实际线速度: ")); Serial.print(realLinearVel); Serial.println(F(" m/s"));
        Serial.print(F("实际角速度: ")); Serial.print(realAngularVel); Serial.println(F(" rad/s"));
        Serial.print(F("ACK已发送: ")); Serial.println(sequence);
      } else if (currentMode == MODE_DATA_RATE_TEST) {
        // 在数据率测试中，只定期打印进度
        if (rfManager.getStats().goodPackets % 100 == 0) {
          Serial.print(F("已接收 "));
          Serial.print(rfManager.getStats().goodPackets);
          Serial.println(F(" 个有效数据包"));
        }
      } else if (currentMode == MODE_PACKET_LOSS_TEST) {
        // 在丢包率测试中，只定期打印进度
        TransmissionStats stats = rfManager.getStats();
        if (stats.goodPackets % 10 == 0) {
          Serial.print(F("已接收 "));
          Serial.print(stats.goodPackets);
          Serial.println(F(" 个有效数据包"));
        }
      }
    } else {
      rfManager.markBadPacket();
      if (currentMode == MODE_NORMAL) {
        Serial.println(F("数据包解析失败!"));
        // 调试 - 打印详细数据包内容以便分析
        dataProcessor.printPacketDetails(actualData, min(20, actualDataLength));
      }
    }
    
    if (currentMode == MODE_NORMAL) {
      Serial.println(F("==== 数据处理完成 ===="));
    }
  }
}