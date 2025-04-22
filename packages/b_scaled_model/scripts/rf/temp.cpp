#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Servo.h>
#include <printf.h>

// 常量定义
#define FRAME_HEADER1   0xAA  // 主帧头标识
#define FRAME_HEADER2   0x7E  // 备用帧头标识
#define FRAME_TAIL      0x55  // 帧尾标识
#define MOTOR_PIN 11        // 电机控制引脚 - 使用Timer2的OC2A引脚(Nano上是D11)
#define SERVO_PIN 9         // 舵机控制引脚
#define RF24_CE_PIN 7       // RF24 CE引脚
#define RF24_CSN_PIN 8      // RF24 CSN引脚
#define TIMEOUT_MS 5000     // 通信超时时间(毫秒)

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

// 电机控制类 (使用Timer2实现300Hz)
class MotorController {
private:
  uint8_t pin;           // 电机控制引脚
  int pulseWidth;        // 当前脉冲宽度(μs)
  const int DEFAULT_PULSE = 1500;  // 中位值(μs)
  const int FORWARD_THRESHOLD = 1350; // 前进临界值(μs)
  const int MIN_RUNNING_PULSE = 1320; // 低速运转标准(μs)
  
  // 将脉冲宽度转换为Timer2的计数值
  // Arduino Nano与UNO的Timer2是8位计数器
  int pulseToTimerCount(int pulse) {
    // 限制脉冲宽度
    pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);
    
    // 使用方法2：固定频率PWM
    // 我们将1000-2000μs映射到0-255之间的值
    // 但因为我们需要特定的占空比来模拟ESC所需脉冲
    // 所以采用比例关系: 255 * (pulse/3333) 
    // 3333μs是300Hz的周期
    // 返回映射后的脉冲计数值
    return map(pulse, MIN_PULSE, MAX_PULSE, 76, 153);
  }

public:
  // 将这些常量改为公有，以便在类外部访问
  const int MIN_PULSE = 1000;      // 最小脉冲宽度(μs)
  const int MAX_PULSE = 2000;      // 最大脉冲宽度(μs)

  // 构造函数，初始化电机控制器
  MotorController(uint8_t pwmPin) : pin(pwmPin), pulseWidth(DEFAULT_PULSE) {}
  
  // 初始化定时器和PWM设置 - 使用Timer2实现PWM
  void begin() {
    // 配置引脚为输出
    pinMode(pin, OUTPUT);
    
    // 关闭全局中断，配置定时器
    cli();
    
    // Timer2配置 - Fast PWM模式
    // WGM22=0, WGM21=1, WGM20=1 (Fast PWM, TOP=0xFF)
    TCCR2A = (1 << WGM21) | (1 << WGM20);
    TCCR2B = 0;
    
    // 设置预分频器，使用适合300Hz的值
    // 16MHz / 256 / 210 ≈ 297Hz，接近我们想要的300Hz
    // CS22=1, CS21=1, CS20=0 (分频系数为256)
    TCCR2B |= (1 << CS22) | (1 << CS21);
    
    // 根据引脚选择适当的比较输出模式
    if (pin == 11) {  // OC2A (D11 on Nano)
      TCCR2A |= (1 << COM2A1);
    } else if (pin == 3) {  // OC2B (D3 on Nano)
      TCCR2A |= (1 << COM2B1);
    }
    
    // 设置默认占空比
    setPulseWidth(DEFAULT_PULSE);
    
    // 重新开启全局中断
    sei();
  }
  
  // 设置脉冲宽度
  void setPulseWidth(int width) {
    // 确保脉冲宽度在有效范围内
    width = constrain(width, MIN_PULSE, MAX_PULSE);
    pulseWidth = width;
    
    // 转换为Timer2的计数值并设置OCR2x
    int timerValue = pulseToTimerCount(width);
    
    // 根据引脚更新相应的OCR寄存器
    if (pin == 11) {  // OC2A
      OCR2A = timerValue;
    } else if (pin == 3) {  // OC2B
      OCR2B = timerValue;
    }
  }
  
  // 复位到中位
  void reset() {
    setPulseWidth(DEFAULT_PULSE);
  }
  
  // 获取当前脉冲宽度
  int getPulseWidth() {
    return pulseWidth;
  }
  
  // 获取默认脉冲宽度
  int getDefaultPulse() {
    return DEFAULT_PULSE;
  }
  
  // 设置速度，输入范围为-1.0至2.7 m/s
  void setSpeed(float speedMps) {
    int mappedPulse = mapSpeedToPulse(speedMps);
    setPulseWidth(mappedPulse);
  }
  
  // 速度映射函数：将速度(m/s)映射到脉冲宽度(μs)
  int mapSpeedToPulse(float speedMps) {
    // 速度范围：-1.0到2.7 m/s
    // 脉冲范围：1000到2000 μs
    // 正速度表示前进(对应1000-1500)，负速度表示后退(对应1500-2000)
    
    if (speedMps == 0) {
      // 零速度对应中位
      return DEFAULT_PULSE;
    } else if (speedMps > 0) {
      // 前进 (正速度映射到1000-1500)
      // 限制最大前进速度为2.7 m/s
      float limitedSpeed = constrain(speedMps, 0, 2.7);
      // 映射到脉冲值，正速度越大，脉冲越小
      return map(limitedSpeed * 1000, 0, 2700, DEFAULT_PULSE, MIN_PULSE);
    } else {
      // 后退 (负速度映射到1500-2000)
      // 限制最大后退速度为-1.0 m/s
      float limitedSpeed = constrain(speedMps, -1.0, 0);
      // 映射到脉冲值，负速度越大(绝对值越大)，脉冲越大
      return map(limitedSpeed * 1000, 0, -1000, DEFAULT_PULSE, MAX_PULSE);
    }
  }
  
  // 判断当前是否在前进
  bool isMovingForward() {
    return pulseWidth < DEFAULT_PULSE;
  }
  
  // 判断当前是否在后退
  bool isMovingBackward() {
    return pulseWidth > DEFAULT_PULSE;
  }
  
  // 判断电机是否在运行
  bool isRunning() {
    return (pulseWidth < MIN_RUNNING_PULSE) || (pulseWidth > DEFAULT_PULSE);
  }
};

// 自定义舵机控制类
class CustomServo {
private:
  Servo servo;         // 底层舵机对象
  int pin;             // 舵机连接的引脚
  int currentPosition; // 当前舵机位置 (0-100)
  
  // 修改的映射函数 - 将0-100的位置直接映射到0-100度角度
  int mapToServoAngle(int position) {
    // 直接映射，确保p50对应50度
    return position;
  }
  
public:
  // 构造函数
  CustomServo() {
    currentPosition = 50; // 默认为中位
  }
  
  // 初始化函数
  void init(int servoPin) {
    pin = servoPin;
    servo.attach(pin);
    // 初始化时设置到中位(50)
    setPosition(50);
  }
  
  // 设置位置 (0-100)
  void setPosition(int position) {
    // 确保位置在有效范围内
    position = constrain(position, 0, 100);
    currentPosition = position;
    
    // 将0-100的位置映射为舵机角度写入
    int angle = mapToServoAngle(position);
    servo.write(angle);
  }
  
  // 根据角度偏移设置位置 (-30 到 +30 映射到 0-100)
  // 负值表示左转，正值表示右转
  void setAngleOffset(int angleOffset) {
    // 确保角度偏移在-30到+30范围内
    angleOffset = constrain(angleOffset, -30, 30);
    
    // 计算位置：从中位50开始，向左(-30)对应100，向右(+30)对应0
    // 所以-30->100, 0->50, +30->0
    int position = map(angleOffset, -30, 30, 100, 0);
    
    // 设置位置
    setPosition(position);
  }
  
  // 获取当前位置
  int getCurrentPosition() {
    return currentPosition;
  }
};

// 全局可修改配置
// RF24地址配置（5字节地址）
const uint64_t RF24_ADDRESS = 0xF0F0F0F066LL;  // 通信地址，发送和接收需使用相同地址

// RF24通道配置
const uint8_t RF24_CHANNEL = 115;  

// RF24数据速率配置
const rf24_datarate_e RF24_DATA_RATE = RF24_1MBPS;

// RF24发射功率配置
const rf24_pa_dbm_e RF24_POWER_LEVEL = RF24_PA_MAX;

// 帧头状态枚举类型
enum FrameHeaderStatus {
  HEADERS_NONE = 0,      // 无帧头
  HEADER1_ONLY = 1,      // 只有主帧头
  HEADER2_ONLY = 2,      // 只有备用帧头
  HEADERS_BOTH = 3       // 两个帧头都存在
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
  
  // 获取底层RF24对象（如果需要直接访问）
  RF24* getRF24() {
    return &radio;
  }
};

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
        return false;
    }
    
    // 查找第一个帧尾标识
    bool foundTail = findFrameTail(data, dataOffset, dataLength, &tailPosition);
    if (!foundTail) {
      return false;
    }
    
    // 帧尾前一个字节是校验和
    checksumPosition = tailPosition - 1;
    if (checksumPosition <= dataOffset) {
      return false;
    }
    
    // 验证帧尾
    if (data[tailPosition] != FRAME_TAIL) {
      return false;
    }
    
    // 验证校验和 (不包括校验和字段和帧尾)
    uint8_t expectedChecksum = data[checksumPosition];
    uint8_t calcChecksum = calculateChecksum(data, headerStatus, dataOffset, checksumPosition);
    
    if (calcChecksum != expectedChecksum) {
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
      return false;
    }
    
    // 验证数据包并获取数据偏移量
    uint8_t dataOffset;
    if (!validateMotionData(data, headerStatus, dataOffset, dataLength)) {
      return false;
    }
    
    // 确保是运动数据包
    if (data[dataOffset] != PACKET_MOTION) {
      return false;
    }
    
    // 计算各字段偏移量
    int deviceIdPos = dataOffset + 1;
    
    // 确保索引不会越界
    if (deviceIdPos >= dataLength) {
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
};

// 创建全局实例
MotorController motor(MOTOR_PIN);
CustomServo steering;
RF24Manager rfManager(RF24_CE_PIN, RF24_CSN_PIN);
DataProcessor dataProcessor;
byte receivedData[32];

// 全局变量
unsigned long lastCommandTime = 0;  // 最后一次接收命令的时间
bool systemEnabled = false;         // 系统启用状态
uint8_t lastSequence = 255;         // 上一个序列号

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
  Serial.println(F("RF24车辆控制初始化"));
  
  // 按照RF24->Servo->Motor的顺序初始化
  
  // 1. 首先初始化RF模块
  if (rfManager.initialize()) {
    // 设置为接收模式
    rfManager.setupReceiver();
    Serial.println(F("RF24接收器就绪"));
  } else {
    Serial.println(F("RF24初始化失败!"));
  }
  
  delay(50); // 给RF24一些时间稳定
  
  // 2. 初始化舵机
  steering.init(SERVO_PIN);
  steering.setPosition(50);  // 中位
  Serial.println(F("舵机初始化完成"));
  
  delay(50); // 等待舵机初始化完成
  
  // 3. 最后初始化电机控制器 - 使用Timer2实现300Hz PWM
  motor.begin();
  delay(10);
  motor.reset();  // 设置为中位值
  Serial.println(F("电机初始化完成"));
  
  // 系统未激活
  systemEnabled = false;
}

void loop() {
  // 读取数据
  uint8_t size;
  if (rfManager.receiveData(receivedData, &size)) {
    // 直接检查从索引1开始的数据 - 不关心buffer[0]
    uint8_t* actualData = &receivedData[1];
    int actualDataLength = size - 1;  // 实际数据长度（不包括buffer[0]）
    
    // 解析数据
    uint8_t deviceId;
    int16_t linearVel, angularVel;
    int16_t linearAcc, angularAcc;
    uint8_t sequence;
    
    // 传递从receivedData[1]开始的数据给解析函数
    bool success = dataProcessor.unpackMotionData(actualData, actualDataLength,
                                     deviceId, linearVel, angularVel,
                                     linearAcc, angularAcc, sequence);
                                     
    // 仅输出序列号和解包状态 
    if (success) {
      // 仅当序列号变化时输出
      if (sequence != lastSequence) {
        Serial.print(F("SEQ:"));
        Serial.print(sequence);
        Serial.println(F(" OK"));
        lastSequence = sequence;
      }
      
      // 发送ACK
      rfManager.sendAck(sequence);
      
      // 更新最后命令时间
      lastCommandTime = millis();
      systemEnabled = true;
      
      // 处理运动控制命令
      processMotionCommand(linearVel, angularVel);
    } else {
      Serial.println(F("解包失败"));
    }
  }
  
  // 检查通信超时
  checkTimeout();
}

// 处理运动控制命令
void processMotionCommand(int16_t linearVelocity, int16_t angularVelocity) {
  // 将原始数据转换为适合车辆的控制值
  
  // 线速度控制 (范围: -1000到2700，对应-1.0到2.7 m/s)
  float speedMps = linearVelocity / 1000.0;  // 转换为m/s
  motor.setSpeed(speedMps);
  
  // 方向控制 (范围: -30到30度)
  // 假设angularVelocity的范围是-3000到3000，映射到-30到30度
  int steeringAngle = map(angularVelocity, -3000, 3000, -30, 30);
  steering.setAngleOffset(steeringAngle);
}

// 检查通信超时
void checkTimeout() {
  if (systemEnabled && (millis() - lastCommandTime > TIMEOUT_MS)) {
    // 超过5秒未收到新命令，停止车辆
    motor.reset();  // 电机回到中位
    steering.setPosition(50);  // 舵机回到中位
    systemEnabled = false;
    Serial.println(F("通信超时，车辆已停止"));
  }
}