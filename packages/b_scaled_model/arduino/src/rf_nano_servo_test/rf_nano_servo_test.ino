#include <Servo.h>

// 创建舵机对象
Servo myServo;

// 定义舵机连接引脚
// 使用支持PWM的9号引脚 (PB1/OC1A)
const int servoPin = 9;

// 舵机位置变量
int pos = 0;

// 基于舵机规格的常量
//在这份代码中，50是中位，0是向右极值，100是向左极值
/*
自定义servo类，开机init到50°，需要一个转动角度函数，将（-30，30）的角度映射到0-100；50为中值（全速转动）
*/
const int MAX_ANGLE = 120; // 基于规格的最大角度(120°±5°)
const int MIN_ANGLE = 0;   // 最小角度

// 串口输入相关变量
String inputString = "";      // 存储串口输入的字符串
boolean stringComplete = false;  // 标记串口输入是否完成

void setup() {
  // 初始化串口通信，用于调试和控制
  Serial.begin(9600);
  Serial.println("RF-NANO 舵机控制程序已启动");
  Serial.println("请输入0-120之间的数字来控制舵机角度");
  Serial.println("输入'c'可将舵机置于中心位置");
  
  // 连接舵机到指定引脚
  myServo.attach(servoPin);
  
  // 初始化舵机至中心位置
  myServo.write(MAX_ANGLE / 2);
  delay(500);
  
  // 初始化串口输入字符串
  inputString.reserve(10);
}

void loop() {
  // 当收到完整的串口输入时处理
  if (stringComplete) {
    // 输入处理
    processInput();
    
    // 清空串口输入变量，准备接收下一次输入
    inputString = "";
    stringComplete = false;
  }
}

// 处理串口输入的函数
void processInput() {
  inputString.trim(); // 移除首尾空格
  
  // 检查是否输入了 'c' 命令（中心位置）
  if (inputString.equals("c") || inputString.equals("C")) {
    Serial.println("移动舵机到中心位置");
    pos = MAX_ANGLE / 2;
    myServo.write(pos);
    Serial.print("当前角度: ");
    Serial.println(pos);
    return;
  }
  
  // 尝试将输入转换为整数
  int angle = inputString.toInt();
  
  // 检查输入是否在有效范围内
  if (angle >= MIN_ANGLE && angle <= MAX_ANGLE) {
    Serial.print("移动舵机到角度: ");
    Serial.println(angle);
    
    // 平滑移动到目标角度
    smoothMove(pos, angle);
    pos = angle;
  } else {
    Serial.print("无效角度! 请输入范围在 ");
    Serial.print(MIN_ANGLE);
    Serial.print(" 到 ");
    Serial.print(MAX_ANGLE);
    Serial.println(" 之间的数字。");
  }
}

// 平滑移动舵机的函数
void smoothMove(int startPos, int endPos) {
  if (startPos < endPos) {
    // 递增移动
    for (int i = startPos; i <= endPos; i++) {
      myServo.write(i);
      delay(5);
    }
  } else {
    // 递减移动
    for (int i = startPos; i >= endPos; i--) {
      myServo.write(i);
      delay(5);
    }
  }
  Serial.print("当前角度: ");
  Serial.println(endPos);
}

// 当串口有数据可读时触发的函数
void serialEvent() {
  while (Serial.available()) {
    // 读取下一个字符
    char inChar = (char)Serial.read();
    
    // 如果是换行符或回车符，标记输入完成
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } else {
      // 否则将字符添加到输入字符串
      inputString += inChar;
    }
  }
}