#include <Arduino.h>

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
  
  // 初始化定时器和PWM设置
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
    
    // 设置TOP值 (PWM周期)
    // 300Hz频率，TOP = 2000000 / 300 = 6666.666...
    ICR1 = 6667;
    
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

// 全局电机控制器实例
MotorController motor(10);  // 使用D10引脚

String inputString = "";    // 用于存储串口输入的字符串
boolean stringComplete = false;  // 标记输入是否完成

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
  Serial.println("Starting Motor Controller...");
  Serial.println("Enter pulse width (1000-2000us) or 's' to reset to 1500us:");
  
  // 初始化电机控制器
  motor.begin();
  
  Serial.println("Motor Controller started.");
  Serial.print("Current pulse width: ");
  Serial.print(motor.getPulseWidth());
  Serial.println("us");
}

void loop() {
  // 检查是否有新的串口输入
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // 如果收到换行符，表示输入完成
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } else {
      // 否则，将字符添加到输入字符串
      inputString += inChar;
    }
  }

  // 如果输入完成，处理输入
  if (stringComplete) {
    // 检查是否输入了's'，如果是则重置为中位
    if (inputString.equals("s") || inputString.equals("S")) {
      motor.reset();
      Serial.print("Reset to default pulse width: ");
      Serial.print(motor.getPulseWidth());
      Serial.println("us");
    } 
    // 检查是否输入了速度值(带m/s后缀)
    else if (inputString.endsWith("m/s")) {
      // 移除"m/s"后缀并尝试解析速度值
      String speedStr = inputString.substring(0, inputString.length() - 3);
      float speed = speedStr.toFloat();
      motor.setSpeed(speed);
      Serial.print("Speed set to: ");
      Serial.print(speed);
      Serial.print(" m/s, Pulse width: ");
      Serial.print(motor.getPulseWidth());
      Serial.println("us");
    }
    // 否则尝试将输入解析为直接的脉冲宽度数值
    else {
      int newPulse = inputString.toInt();
      
      // 现在我们可以直接访问公有的MIN_PULSE和MAX_PULSE常量
      if (newPulse >= motor.MIN_PULSE && newPulse <= motor.MAX_PULSE) {
        motor.setPulseWidth(newPulse);
        Serial.print("Pulse width updated to: ");
        Serial.print(motor.getPulseWidth());
        Serial.println("us");
        
        // 显示当前运动状态
        if (motor.isMovingForward()) {
          Serial.println("Motor is moving forward.");
        } else if (motor.isMovingBackward()) {
          Serial.println("Motor is moving backward.");
        } else {
          Serial.println("Motor is stopped.");
        }
      } else {
        Serial.println("Invalid input! Please enter a value between 1000 and 2000us.");
      }
    }
    
    // 清空输入字符串，准备下一次输入
    inputString = "";
    stringComplete = false;
    Serial.println("Enter pulse width (1000-2000us), speed (-1.0 to 2.7m/s), or 's' to reset:");
  }
}