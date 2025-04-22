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

// RF24通信类封装
class RF24Manager {
private:
  RF24 radio;
  byte buffer[32];
  
public:
  // 构造函数
  RF24Manager(uint8_t cePin, uint8_t csnPin) : radio(cePin, csnPin) {}
  
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
  
  // 打印RF24详细信息
  void printDetails() {
    radio.printDetails();
  }
  
  // 获取底层RF24对象（如果需要直接访问）
  RF24* getRF24() {
    return &radio;
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

// 创建RF24Manager实例 (CE引脚=7, CSN引脚=8)
RF24Manager rfManager(7, 8);
DataProcessor dataProcessor;
byte receivedData[32];

void setup() {
  Serial.begin(115200);
  printf_begin();
  Serial.println(F("RF-NANO v4.0 运动数据接收测试 - 双帧头增强版"));
  Serial.println(F("-------------------------------"));

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
}

void loop() {
  // 读取数据
  uint8_t size;
  if (rfManager.receiveData(receivedData, &size)) {
    // 打印接收到的原始数据
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
    
    // 直接检查从索引1开始的数据 - 不关心buffer[0]
    uint8_t* actualData = &receivedData[1];
    int actualDataLength = size - 1;  // 实际数据长度（不包括buffer[0]）
    
    // 从数据处理器中获取帧头状态
    FrameHeaderStatus headerStatus = dataProcessor.detectHeaderStatus(actualData);
    
    // 根据帧头状态进行处理
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
      case HEADERS_NONE:
        Serial.println(F("未检测到任何帧头，数据无效"));
        Serial.println(F("==== 数据处理完成 ===="));
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
      // 解析成功，输出解析结果
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
    } else {
      Serial.println(F("数据包解析失败!"));
      // 调试 - 打印详细数据包内容以便分析
      dataProcessor.printPacketDetails(actualData, min(20, actualDataLength));
    }
    
    Serial.println(F("==== 数据处理完成 ===="));
  }
  
}