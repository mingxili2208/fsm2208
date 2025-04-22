#include <Arduino.h>
#include <ServoTimer2.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <printf.h>

// RF24协议常量定义
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

// 帧头状态枚举类型
enum FrameHeaderStatus {
  HEADERS_NONE = 0,      // 无帧头
  HEADER1_ONLY = 1,      // 只有主帧头
  HEADER2_ONLY = 2,      // 只有备用帧头
  HEADERS_BOTH = 3       // 两个帧头都存在
};

// 全局RF24配置
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

// 自定义舵机控制类
class CustomServo {
private:
  ServoTimer2 servo;
  int pin;
  int currentAngle;
  int currentPulseWidth;
  
  const int CENTER_PULSE = 1000;
  const int LEFT_PULSE = 1300;
  const int RIGHT_PULSE = 700;
  const int LEFT_ANGLE = -25;
  const int RIGHT_ANGLE = 25;
  
  int angleToPulseWidth(int angle) {
    angle = constrain(angle, LEFT_ANGLE, RIGHT_ANGLE);
    if (angle < 0) {
      return map(angle, 0, LEFT_ANGLE, CENTER_PULSE, LEFT_PULSE);
    } else {
      return map(angle, 0, RIGHT_ANGLE, CENTER_PULSE, RIGHT_PULSE);
    }
  }
  
  int pulseWidthToAngle(int pulseWidth) {
    pulseWidth = constrain(pulseWidth, RIGHT_PULSE, LEFT_PULSE);
    if (pulseWidth > CENTER_PULSE) {
      return map(pulseWidth, CENTER_PULSE, LEFT_PULSE, 0, LEFT_ANGLE);
    } else {
      return map(pulseWidth, CENTER_PULSE, RIGHT_PULSE, 0, RIGHT_ANGLE);
    }
  }
  
public:
  CustomServo() {
    currentAngle = 0;
    currentPulseWidth = CENTER_PULSE;
  }
  
  void init(int servoPin) {
    pin = servoPin;
    servo.attach(pin);
    setAngle(0);
    Serial.println(F("舵机初始化完成"));
  }
  
  void setPulseWidth(int pulseWidth) {
    pulseWidth = constrain(pulseWidth, RIGHT_PULSE, LEFT_PULSE);
    currentPulseWidth = pulseWidth;
    servo.write(pulseWidth);
    currentAngle = pulseWidthToAngle(pulseWidth);
    
    Serial.print(F("舵机PW:"));
    Serial.print(pulseWidth);
    Serial.print(F(" 角:"));
    Serial.println(currentAngle);
  }
  
  void setAngle(int angle) {
    angle = constrain(angle, LEFT_ANGLE, RIGHT_ANGLE);
    currentAngle = angle;
    int pulseWidth = angleToPulseWidth(angle);
    currentPulseWidth = pulseWidth;
    servo.write(pulseWidth);
    
    Serial.print(F("舵机角:"));
    Serial.print(angle);
    Serial.print(F(" PW:"));
    Serial.println(pulseWidth);
  }
  
  int getCurrentAngle() {
    return currentAngle;
  }
  
  int getCurrentPulseWidth() {
    return currentPulseWidth;
  }
  
  // 获取舵机角度限制
  int getLeftAngleLimit() {
    return LEFT_ANGLE;
  }
  
  int getRightAngleLimit() {
    return RIGHT_ANGLE;
  }
};

// 电机控制器类
class MotorController {
private:
  uint8_t pin;
  int pulseWidth;
  float prevSpeed; // 记录上一次的速度值
  
  // 电机参数
  const int MIN_PULSE = 1200;         // 最小脉冲宽度
  const int MAX_PULSE = 1700;         // 最大脉冲宽度
  const int NEUTRAL_MIN = 1350;       // 中位区间最小值
  const int NEUTRAL_MAX = 1600;       // 中位区间最大值
  const int DEFAULT_PULSE = 1500;     // 默认中位值（NEUTRAL_MIN和NEUTRAL_MAX的中间值）
  const float MAX_FORWARD_SPEED = 1.0; // 最大前进速度 m/s (根据需求修改为2.7m/s)
  const float MAX_REVERSE_SPEED = -1.0; // 最大后退速度 m/s

public:
  MotorController(uint8_t pwmPin) : pin(pwmPin), pulseWidth(DEFAULT_PULSE), prevSpeed(0.0) {}
  
  void begin() {
    pinMode(pin, OUTPUT);
    cli();
    
    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12);
    TCCR1B |= (1 << CS11);
    
    if (pin == 9) {
      TCCR1A |= (1 << COM1A1);
    } else if (pin == 10) {
      TCCR1A |= (1 << COM1B1);
    }
    
    ICR1 = 6667;
    setPulseWidth(DEFAULT_PULSE);
    sei();
    
    Serial.println(F("电机初始化完成"));
  }
  
  void setPulseWidth(int width) {
    width = constrain(width, MIN_PULSE, MAX_PULSE);
    pulseWidth = width;
    
    int ocrValue = width * 2;
    if (pin == 9) {
      OCR1A = ocrValue;
    } else if (pin == 10) {
      OCR1B = ocrValue;
    }
    
    Serial.print(F("电机PW:"));
    Serial.println(pulseWidth);
  }
  
  void reset() {
    setPulseWidth(DEFAULT_PULSE);
    prevSpeed = 0.0;
    Serial.println(F("电机复位"));
  }
  
  int getPulseWidth() {
    return pulseWidth;
  }
  
  void setSpeed(float speedMps) {
    // 只在速度变化超过阈值时才更新，避免频繁小幅度调整
    if (abs(speedMps - prevSpeed) > 0.05) {
      int mappedPulse = mapSpeedToPulse(speedMps);
      
      // 执行速度变化
      setPulseWidth(DEFAULT_PULSE);
      delay(20);
      setPulseWidth(mappedPulse);
      
      Serial.print(F("电机速度:"));
      Serial.print(speedMps);
      Serial.println(F(" m/s"));
      
      prevSpeed = speedMps;
      printStatus();
    }
  }
  
  int mapSpeedToPulse(float speedMps) {
    // 确保速度在允许范围内
    speedMps = constrain(speedMps, MAX_REVERSE_SPEED, MAX_FORWARD_SPEED);
    
    // 零速度返回中位值
    if (abs(speedMps) < 0.05) {
      return DEFAULT_PULSE;
    }
    
    if (speedMps > 0) {
      // 前进: 从中位值到最小脉冲值(1350->1200)
      // 映射正向速度到前进脉冲范围
      return map(speedMps * 100, 0, MAX_FORWARD_SPEED * 100, NEUTRAL_MIN, MIN_PULSE);
    } else {
      // 后退: 从中位值到最大脉冲值(1600->1700)
      // 映射负向速度到后退脉冲范围
      return map(speedMps * 100, 0, MAX_REVERSE_SPEED * 100, NEUTRAL_MAX, MAX_PULSE);
    }
  }
  
  bool isMovingForward() {
    return pulseWidth < NEUTRAL_MIN;
  }
  
  bool isMovingBackward() {
    return pulseWidth > NEUTRAL_MAX;
  }
  
  bool isRunning() {
    return isMovingForward() || isMovingBackward();
  }
  
  float getCurrentSpeed() {
    if (pulseWidth >= NEUTRAL_MIN && pulseWidth <= NEUTRAL_MAX) {
      return 0.0;
    } else if (pulseWidth < NEUTRAL_MIN) {
      // 前进状态 - 从脉冲宽度反向映射到速度
      return map(pulseWidth, NEUTRAL_MIN, MIN_PULSE, 0, MAX_FORWARD_SPEED * 100) / 100.0;
    } else {
      // 后退状态 - 从脉冲宽度反向映射到速度
      return map(pulseWidth, NEUTRAL_MAX, MAX_PULSE, 0, MAX_REVERSE_SPEED * 100) / 100.0;
    }
  }
  
  void printStatus() {
    Serial.print(F("状态:"));
    if (isMovingForward()) {
      Serial.print(F("前进 "));
      Serial.print(getCurrentSpeed());
      Serial.println(F(" m/s"));
    } else if (isMovingBackward()) {
      Serial.print(F("后退 "));
      Serial.print(getCurrentSpeed());
      Serial.println(F(" m/s"));
    } else {
      Serial.println(F("停止"));
    }
  }
  
  // 获取速度限制
  float getMaxForwardSpeed() {
    return MAX_FORWARD_SPEED;
  }
  
  float getMaxReverseSpeed() {
    return MAX_REVERSE_SPEED;
  }
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
    if (data[i] == FRAME_TAIL && i >= 13) {
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

// 创建控制对象
CustomServo myServo;
MotorController motor(10);      // D10引脚用于电机
RF24Manager rfManager(7, 8);    // CE=7, CSN=8用于RF24
DataProcessor dataProcessor;
byte receivedData[32];

const int SERVO_PIN = 3;        // D3引脚用于舵机

// 通信变量
unsigned long lastDataReceivedTime = 0;
const unsigned long DATA_TIMEOUT = 2000;  // 1秒无数据则停止
bool receivedValidData = false;

// 状态变量
int lastServoAngle = 0;
float lastMotorSpeed = 0.0;

void setup() {
  Serial.begin(115200);
  printf_begin();
  Serial.println(F("RF24通信与电机舵机控制系统"));
  Serial.println(F("-------------------------------"));
  
  // 初始化舵机
  myServo.init(SERVO_PIN);
  
  // 初始化电机
  motor.begin();
  motor.reset();
  
  // 初始化RF24
  if (rfManager.initialize()) {
    // 设置为接收模式
    rfManager.setupReceiver();
    Serial.println(F("RF24接收器初始化成功"));
    rfManager.printDetails();
  } else {
    Serial.println(F("RF24初始化失败"));
  }
  
  Serial.println(F("系统初始化完成，等待接收数据..."));
  Serial.println(F("支持速度范围:-1.0至2.7m/s"));
  Serial.println(F("支持角度范围:-25至25度"));
}

void loop() {
  // 检查RF24数据
  uint8_t size;
  if (rfManager.receiveData(receivedData, &size)) {
    // 记录接收时间
    lastDataReceivedTime = millis();
    receivedValidData = true;
    
    // 直接检查从索引1开始的数据 - 不关心buffer[0]
    uint8_t* actualData = &receivedData[1];
    int actualDataLength = size - 1;  // 实际数据长度（不包括buffer[0]）
    
    // 解析并控制电机舵机
    processRF24Data(actualData, actualDataLength);
  }
  
  // 安全检查 - 如果一段时间内没有接收到数据，则停止电机
  if (receivedValidData && millis() - lastDataReceivedTime > DATA_TIMEOUT) {
    Serial.println(F("\n通信超时，停止运动"));
    motor.reset();
    myServo.setAngle(0);
    receivedValidData = false;
    lastServoAngle = 0;
    lastMotorSpeed = 0.0;
  }
}

// 处理RF24接收的数据并控制电机舵机
void processRF24Data(uint8_t* data, int dataLength) {
  uint8_t deviceId;
  int16_t linearVel, angularVel;
  int16_t linearAcc, angularAcc;
  uint8_t sequence;
  
  if (dataProcessor.unpackMotionData(data, dataLength,
                                   deviceId, linearVel, angularVel,
                                   linearAcc, angularAcc, sequence)) {
    
    // 转换为实际物理值 (mm/s转换为m/s，mrad/s转换为rad/s然后转换为角度)
    float speed = linearVel / 1000.0;  // 转换为m/s
    
    // 从角速度计算角度 - 将弧度转换为角度
    // 角速度是弧度/秒，需要转换为我们需要的角度范围(-25到25度)
    float angularRadians = angularVel / 1000.0;  // 转换为rad/s
    
    // 将弧度/秒转换为角度，并映射到舵机角度范围
    // 假设±1.0弧度/秒映射到±25度舵机角度
    int servoAngle = constrain(angularRadians * 25.0, myServo.getLeftAngleLimit(), myServo.getRightAngleLimit());
    
    // 限制速度范围
    speed = constrain(speed, motor.getMaxReverseSpeed(), motor.getMaxForwardSpeed());
    
    // 仅当状态发生变化时才输出信息和更新控制
    bool servoChanged = (servoAngle != lastServoAngle);
    bool motorChanged = (abs(speed - lastMotorSpeed) > 0.05);
    
    if (servoChanged || motorChanged) {
      Serial.println(F("\n==== 接收到新控制数据 ===="));
      Serial.print(F("设备ID: ")); Serial.println(deviceId);
      Serial.print(F("序列号: ")); Serial.println(sequence);
      
      // 输出速度信息
      if (motorChanged) {
        Serial.print(F("原始线速度: ")); Serial.print(linearVel); Serial.println(F(" (1/1000 m/s)"));
        Serial.print(F("转换后速度: ")); Serial.print(speed); Serial.println(F(" m/s"));
        motor.setSpeed(speed);
        lastMotorSpeed = speed;
      }
      
      // 输出角度信息
      if (servoChanged) {
        Serial.print(F("原始角速度: ")); Serial.print(angularVel); Serial.println(F(" (1/1000 rad/s)"));
        Serial.print(F("转换后角度: ")); Serial.print(servoAngle); Serial.println(F(" 度"));
        myServo.setAngle(servoAngle);
        lastServoAngle = servoAngle;
      }
      
      Serial.println(F("==== 控制更新完成 ===="));
    }
  } else {
    Serial.println(F("\n数据包解析失败!"));
    // 打印出16进制的指令原文
    Serial.println(F("原始数据内容（十六进制）:"));
    for (int i = 0; i < dataLength; i++) {
      if (data[i] < 16) Serial.print(F("0")); // 补零确保两位十六进制显示
      Serial.print(data[i], HEX);
      Serial.print(F(" "));
      // 每8个字节换行以便阅读
      if ((i + 1) % 8 == 0) Serial.println();
    }
    Serial.println(); // 结束当前行
  }
}