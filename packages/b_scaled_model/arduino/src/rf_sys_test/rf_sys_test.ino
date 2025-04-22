#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Servo.h>
#include <printf.h>

// 常量定义
#define MOTOR_PIN 10        // 电机控制引脚
#define SERVO_PIN 9         // 舵机控制引脚
#define RF24_CE_PIN 7       // RF24 CE引脚
#define RF24_CSN_PIN 8      // RF24 CSN引脚
#define TIMEOUT_MS 5000     // 通信超时时间(毫秒)

// RF24地址配置（5字节地址）
const uint64_t RF24_ADDRESS = 0xF0F0F0F066LL;  // 通信地址，发送和接收需使用相同地址
const uint8_t RF24_CHANNEL = 115;  // RF24通道
const rf24_datarate_e RF24_DATA_RATE = RF24_1MBPS;  // 数据速率
const rf24_pa_dbm_e RF24_POWER_LEVEL = RF24_PA_MAX;  // 发射功率

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

// 有效数据包结构（14字节有效数据）
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
};  // 总计14字节有效数据

// 电机控制类 (修改为330Hz)
class MotorController {
private:
  uint8_t pin;           // 电机控制引脚
  int pulseWidth;        // 当前脉冲宽度(μs)
  const int DEFAULT_PULSE = 1500;  // 中位值(μs)
  const int FORWARD_THRESHOLD = 1350; // 前进临界值(μs)
  const int MIN_RUNNING_PULSE = 1320; // 低速运转标准(μs)

public:
  // 将这些常量改为公有，以便在类外部访问
  const int MIN_PULSE = 1000;      // 最小脉冲宽度(μs)
  const int MAX_PULSE = 2000;      // 最大脉冲宽度(μs)

  // 构造函数，初始化电机控制器
  MotorController(uint8_t pwmPin) : pin(pwmPin), pulseWidth(DEFAULT_PULSE) {}
  
  // 初始化定时器和PWM设置 - 修改为330Hz
  void begin() {
    // 配置引脚为输出
    pinMode(pin, OUTPUT);
    
    // 关闭全局中断，配置定时器
    cli();
    
    // Timer 1 配置 - 快速PWM模式
    // WGM13 = 1, WGM12 = 1, WGM11 = 1, WGM10 = 0
    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12);
    
    // 设置预分频器为 8
    // 时钟频率 = 16MHz / 8 = 2MHz
    TCCR1B |= (1 << CS11);
    
    // 设置非反相PWM模式
    // 根据引脚选择合适的COM寄存器位
    if (pin == 9) {
      // OC1A (D9)
      TCCR1A |= (1 << COM1A1);
    } else if (pin == 10) {
      // OC1B (D10)
      TCCR1A |= (1 << COM1B1);
    }
    
    // 设置TOP值 (PWM周期) - 修改为330Hz
    // 330Hz频率，TOP = 2000000 / 330 = 6060.606...
    ICR1 = 6061;
    
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
    
    // 计算OCR值 (2MHz时钟，每μs对应2个时钟周期)
    int ocrValue = width * 2;
    
    // 根据引脚更新相应的OCR寄存器
    if (pin == 9) {
      OCR1A = ocrValue;
    } else if (pin == 10) {
      OCR1B = ocrValue;
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

// 自定义舵机控制类 (保持不变)
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

// 修改后的RF24通信类封装，适配nRF24L01协议
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
      
      // 设置动态有效载荷长度
      radio.enableDynamicPayloads();
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
  
  // 接收数据包 - 正确处理buffer[0]
  bool receivePacket(uint8_t* payload) {
    if (!radio.available()) {
      return false;
    }
    
    // 获取动态有效载荷长度
    uint8_t payloadSize = radio.getDynamicPayloadSize();
    
    // 读取整个数据包（动态大小）
    uint8_t buffer[32]; // 最大32字节
    radio.read(buffer, payloadSize);
    
    // 检查buffer[0]必须等于0x1F(31)
    if (buffer[0] != 0x1F) {
      return false;
    }
    
    // 将有效数据从buffer[1]开始复制到payload
    for (uint8_t i = 0; i < 31; i++) {
      payload[i] = buffer[i+1];
    }
    
    return validateMotionData(payload);
  }
  
  // 验证MotionData数据包
  bool validateMotionData(const uint8_t* payload) {
    const MotionData* data = (const MotionData*)payload;
    
    // 检查帧头和帧尾
    if (data->header != FRAME_HEADER || data->tail != FRAME_TAIL) {
      return false;
    }
    
    // 验证校验和 (不包括校验和字段和帧尾)
    uint8_t calcChecksum = calculateChecksum(payload, sizeof(MotionData) - 2);
    if (calcChecksum != data->checksum) {
      return false;
    }
    
    return true;
  }
  
  // 解析运动数据包
  bool unpackMotionData(const uint8_t* payload,
                      uint8_t& deviceId,
                      int16_t& linearVel, int16_t& angularVel,
                      int16_t& linearAcc, int16_t& angularAcc,
                      uint8_t& sequence) {
    const MotionData* data = (const MotionData*)payload;
    
    // 确保是运动数据包
    if (data->type != PACKET_MOTION) {
      return false;
    }
    
    // 提取数据
    deviceId = data->deviceId;
    linearVel = data->linearVelocity;
    angularVel = data->angularVelocity;
    linearAcc = data->linearAccel;
    angularAcc = data->angularAccel;
    sequence = data->sequence;
    
    return true;
  }
  
  // 发送数据包 - 手动设置buffer[0]为0x1F
  bool sendPacket(const uint8_t* payload, size_t size) {
    // 准备发送缓冲区
    uint8_t buffer[32] = {0};
    
    // 第一个字节必须是0x1F
    buffer[0] = 0x1F;
    
    // 复制有效数据到buffer[1]开始的位置
    for (uint8_t i = 0; i < 31 && i < size; i++) {
      buffer[i+1] = payload[i];
    }
    
    // 发送数据
    radio.stopListening();
    bool success = radio.write(buffer, 32);  // 始终发送32字节
    radio.startListening();
    
    return success;
  }
};

// 全局实例
MotorController motor(MOTOR_PIN);  // 电机控制
CustomServo steering;              // 舵机控制
RF24Manager rfManager(RF24_CE_PIN, RF24_CSN_PIN);  // RF24通信管理
uint8_t receivedPayload[31];       // 接收数据缓冲区 (31字节)

// 全局变量
unsigned long lastCommandTime = 0;  // 最后一次接收命令的时间
bool systemEnabled = false;         // 系统启用状态

void setup() {
  // 初始化电机控制器 - 330Hz 1500us
  motor.begin();
  motor.setPulseWidth(1500);  // 确保初始化为1500us
  
  // 初始化舵机
  steering.init(SERVO_PIN);
  
  // 初始化RF24
  if (rfManager.initialize()) {
    rfManager.setupReceiver();
  }
  
  // 系统就绪
  systemEnabled = false;
}

void loop() {
  // 检查RF24是否有新数据
  if (rfManager.receivePacket(receivedPayload)) {
    uint8_t deviceId;
    int16_t linearVel, angularVel;
    int16_t linearAcc, angularAcc;
    uint8_t sequence;
    
    if (rfManager.unpackMotionData(receivedPayload, deviceId, 
                                   linearVel, angularVel,
                                   linearAcc, angularAcc,
                                   sequence)) {
      // 更新最后命令时间
      lastCommandTime = millis();
      systemEnabled = true;
      
      // 处理接收到的控制命令
      processMotionCommand(linearVel, angularVel);
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
  }
}