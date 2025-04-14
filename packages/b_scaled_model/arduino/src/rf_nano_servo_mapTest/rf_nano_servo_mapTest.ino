#include <Servo.h>

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
    Serial.println("舵机初始化完成，设置到中位(50)");
  }
  
  // 设置位置 (0-100)
  void setPosition(int position) {
    // 确保位置在有效范围内
    position = constrain(position, 0, 100);
    currentPosition = position;
    
    // 将0-100的位置直接作为舵机角度写入
    int angle = mapToServoAngle(position);
    servo.write(angle);
    
    Serial.print("舵机位置设置为: ");
    Serial.print(position);
    Serial.print(" (舵机角度: ");
    Serial.print(angle);
    Serial.println("°)");
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
    
    Serial.print("角度偏移: ");
    Serial.print(angleOffset);
    Serial.print(" 映射到位置: ");
    Serial.println(position);
  }
  
  // 获取当前位置
  int getCurrentPosition() {
    return currentPosition;
  }
};

// 创建自定义舵机对象
CustomServo myServo;

// 定义舵机连接引脚 (PWM引脚)
const int SERVO_PIN = 9;  // Arduino Nano上的PWM引脚

// 串口控制变量
String inputString = "";
boolean stringComplete = false;

void setup() {
  // 初始化串口通信
  Serial.begin(9600);
  Serial.println("Arduino Nano 自定义舵机控制系统启动");
  Serial.println("--------------------------------------");
  Serial.println("控制指令:");
  Serial.println("p[0-100] - 直接设置位置 (例如: p75)");
  Serial.println("a[-30,30] - 设置角度偏移 (例如: a15)");
  Serial.println("  负值表示左转，正值表示右转");
  Serial.println("c - 回到中位(50)");
  Serial.println("--------------------------------------");
  
  // 初始化舵机
  myServo.init(SERVO_PIN);
  
  // 为串口输入分配内存
  inputString.reserve(20);
}

void loop() {
  // 当收到完整的串口命令时处理
  if (stringComplete) {
    processCommand();
    // 清空输入字符串，准备接收下一个命令
    inputString = "";
    stringComplete = false;
  }
}

// 处理串口命令
void processCommand() {
  // 移除首尾空格
  inputString.trim();
  
  // 检查命令长度
  if (inputString.length() < 1) {
    Serial.println("无效命令");
    return;
  }
  
  // 获取命令类型
  char cmd = inputString.charAt(0);
  
  // 根据命令类型执行相应操作
  switch (cmd) {
    case 'p': case 'P': // 设置位置
      if (inputString.length() > 1) {
        int pos = inputString.substring(1).toInt();
        myServo.setPosition(pos);
      } else {
        Serial.println("错误: 位置命令需要一个值");
      }
      break;
      
    case 'a': case 'A': // 设置角度偏移
      if (inputString.length() > 1) {
        int angleOffset = inputString.substring(1).toInt();
        myServo.setAngleOffset(angleOffset);
      } else {
        Serial.println("错误: 角度命令需要一个值");
      }
      break;
      
    case 'c': case 'C': // 回到中位
      Serial.println("设置舵机到中位(50)");
      myServo.setPosition(50);
      break;
      
    default:
      Serial.println("未知命令");
      break;
  }
}

// 串口事件处理
void serialEvent() {
  while (Serial.available()) {
    // 获取新字符
    char inChar = (char)Serial.read();
    
    // 如果是换行或回车，将触发命令处理
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } else {
      // 否则添加字符到命令字符串
      inputString += inChar;
    }
  }
}