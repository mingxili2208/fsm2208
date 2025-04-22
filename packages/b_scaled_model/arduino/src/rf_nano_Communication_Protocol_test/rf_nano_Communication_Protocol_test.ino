#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <printf.h>

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

// 通信协议定义
#define FRAME_HEADER    0xAA  // 帧头标识
#define FRAME_TAIL      0x55  // 帧尾标识

// 数据包类型定义
enum PacketType {
  PACKET_MOTION = 0x01,   // 运动数据包（包含线速度、角度等）
  PACKET_COMMAND = 0x02,  // 命令数据包
  PACKET_STATUS = 0x03,   // 状态数据包
  PACKET_ACK = 0x04       // 确认数据包
};

// 有效数据包结构（前14字节）
struct MotionData {
  uint8_t header;            // 帧头 (1字节)
  uint8_t type;              // 包类型 (1字节)
  uint8_t deviceId;          // 设备ID (1字节)
  
  int16_t linearVelocity;    // X方向线速度 (2字节)
  int16_t angularVelocity;   // Z方向角速度 (2字节)
  int16_t linearAccel;       // 线加速度 (2字节)
  int16_t angularAccel;      // 角加速度 (2字节)
  
  uint8_t sequence;          // 序列号 (1字节)
  uint8_t checksum;          // 校验和 (1字节)
  uint8_t tail;              // 帧尾 (1字节)
};  // 总计14字节

// 32字节固定大小的RF24数据包
struct RF24Packet {
  union {
    MotionData motionData;         // 有效数据部分（14字节）
    uint8_t rawData[32];           // 原始数据（32字节）
  };
};

// RF24通信类封装
class RF24Manager {
private:
  RF24 radio;
  uint8_t sequenceCounter;
  
  // 计算校验和
  uint8_t calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++) {
      sum += data[i];
    }
    return sum;
  }
  
public:
  // 构造函数
  RF24Manager(uint8_t cePin, uint8_t csnPin) : radio(cePin, csnPin), sequenceCounter(0) {}
  
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
  
  // 打包运动数据包
  void packMotionData(RF24Packet& packet, uint8_t deviceId,
                     int16_t linearVel, int16_t angularVel,
                     int16_t linearAcc, int16_t angularAcc) {
    // 清空整个32字节数据包
    memset(packet.rawData, 0, sizeof(packet.rawData));
    
    // 填充帧头和帧尾
    packet.motionData.header = FRAME_HEADER;
    packet.motionData.tail = FRAME_TAIL;
    
    // 填充包类型和设备ID
    packet.motionData.type = PACKET_MOTION;
    packet.motionData.deviceId = deviceId;
    
    // 填充速度和加速度数据
    packet.motionData.linearVelocity = linearVel;
    packet.motionData.angularVelocity = angularVel;
    packet.motionData.linearAccel = linearAcc;
    packet.motionData.angularAccel = angularAcc;
    
    // 填充序列号
    packet.motionData.sequence = sequenceCounter++;
    
    // 计算校验和（不包括校验和字段本身和帧尾）
    packet.motionData.checksum = calculateChecksum(packet.rawData, sizeof(MotionData) - 2);
  }
  
  // 验证接收的数据包
  bool validatePacket(const RF24Packet& packet) {
    // 检查帧头和帧尾
    if (packet.motionData.header != FRAME_HEADER || packet.motionData.tail != FRAME_TAIL) {
      return false;
    }
    
    // 验证校验和
    uint8_t calcChecksum = calculateChecksum(packet.rawData, sizeof(MotionData) - 2);
    if (calcChecksum != packet.motionData.checksum) {
      return false;
    }
    
    return true;
  }
  
  // 发送运动数据包
  bool sendMotionData(uint8_t deviceId,
                     int16_t linearVel, int16_t angularVel,
                     int16_t linearAcc, int16_t angularAcc) {
    RF24Packet packet;
    packMotionData(packet, deviceId, linearVel, angularVel, linearAcc, angularAcc);
    return radio.write(packet.rawData, sizeof(packet.rawData));
  }
  
  // 接收数据包
  bool receivePacket(RF24Packet& packet) {
    if (!radio.available()) {
      return false;
    }
    
    // 从RF24读取完整的32字节数据包
    radio.read(packet.rawData, sizeof(packet.rawData));
    
    // 验证数据包
    return validatePacket(packet);
  }
  
  // 解析运动数据包
  bool unpackMotionData(const RF24Packet& packet,
                       uint8_t& deviceId,
                       int16_t& linearVel, int16_t& angularVel,
                       int16_t& linearAcc, int16_t& angularAcc,
                       uint8_t& sequence) {
    // 确保是运动数据包
    if (packet.motionData.type != PACKET_MOTION) {
      return false;
    }
    
    // 提取数据
    deviceId = packet.motionData.deviceId;
    linearVel = packet.motionData.linearVelocity;
    angularVel = packet.motionData.angularVelocity;
    linearAcc = packet.motionData.linearAccel;
    angularAcc = packet.motionData.angularAccel;
    sequence = packet.motionData.sequence;
    
    return true;
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

// 创建RF24Manager实例 (CE引脚=7, CSN引脚=8)
RF24Manager rfManager(7, 8);
RF24Packet receivedPacket;

void setup() {
  Serial.begin(115200);
  printf_begin();
  Serial.println(F("RF-NANO v4.0 Receive Test"));

  // 初始化RF模块 - 使用全局配置的默认参数
  if (rfManager.initialize()) {
    // 设置为接收模式 - 使用全局配置的默认地址
    rfManager.setupReceiver();
    Serial.println("Receive Setup Initialized");
    rfManager.printDetails();
  } else {
    Serial.println("RF24 initialization failed");
  }
  
  delay(500);
}

void loop() {
  // 读取并解析数据包
  if (rfManager.receivePacket(receivedPacket)) {
    uint8_t deviceId;
    int16_t linearVel, angularVel;
    int16_t linearAcc, angularAcc;
    uint8_t sequence;
    
    if (rfManager.unpackMotionData(receivedPacket, deviceId, 
                                  linearVel, angularVel,
                                  linearAcc, angularAcc,
                                  sequence)) {
      // 打印接收到的数据
      Serial.println("Received Motion Data:");
      Serial.print("Device ID: "); Serial.println(deviceId);
      Serial.print("Sequence: "); Serial.println(sequence);
      
      Serial.print("Linear Velocity: "); Serial.println(linearVel);
      Serial.print("Angular Velocity: "); Serial.println(angularVel);
      Serial.print("Linear Acceleration: "); Serial.println(linearAcc);
      Serial.print("Angular Acceleration: "); Serial.println(angularAcc);
      Serial.println();
    }
  }
  
  // 如果需要发送数据的示例
  // rfManager.setupTransmitter();  // 切换到发送模式
  // rfManager.sendMotionData(1, 100, 45, 10, 5);  // 设备ID, 线速度, 角速度, 线加速度, 角加速度
  // rfManager.setupReceiver();     // 切换回接收模式
}