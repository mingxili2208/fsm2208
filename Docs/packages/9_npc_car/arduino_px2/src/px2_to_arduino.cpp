#include <PS2X_lib.h>

// 定义 PS2 控制器引脚
#define PS2_DAT  13
#define PS2_CMD  11
#define PS2_SEL  10
#define PS2_CLK  12

// 定义模式
#define pressures   false
#define rumble      false

PS2X ps2x;  // 创建 PS2 控制器对象

int error = 0;
byte type = 0;
byte vibrate = 0;
String lastDataPacket = "";  // 存储上一次发送的数据包

void setup() {
  Serial.begin(9600);  // 初始化串口通信
  delay(300);  // 给无线 PS2 模块启动时间

  // 配置手柄
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (error == 0) {
    Serial.println("Controller configured successfully.");
  } else {
    Serial.print("Controller configuration failed with error: ");
    Serial.println(error);
    while (true);  // 停止程序
  }

  type = ps2x.readType();  // 读取手柄类型
}

void loop() {
  if (error != 0) return;  // 如果手柄未连接，跳过循环

  ps2x.read_gamepad(false, vibrate);  // 读取手柄状态

  // 构造数据包：发送按键和摇杆信息
  String dataPacket = "";

  // 按键状态
  dataPacket += "START:" + String(ps2x.Button(PSB_START)) + ",";
  dataPacket += "SELECT:" + String(ps2x.Button(PSB_SELECT)) + ",";
  dataPacket += "UP:" + String(ps2x.Button(PSB_PAD_UP)) + ",";
  dataPacket += "DOWN:" + String(ps2x.Button(PSB_PAD_DOWN)) + ",";
  dataPacket += "LEFT:" + String(ps2x.Button(PSB_PAD_LEFT)) + ",";
  dataPacket += "RIGHT:" + String(ps2x.Button(PSB_PAD_RIGHT)) + ",";
  dataPacket += "SQUARE:" + String(ps2x.Button(PSB_SQUARE)) + ",";
  dataPacket += "CROSS:" + String(ps2x.Button(PSB_CROSS)) + ",";
  dataPacket += "CIRCLE:" + String(ps2x.Button(PSB_CIRCLE)) + ",";
  dataPacket += "TRIANGLE:" + String(ps2x.Button(PSB_TRIANGLE)) + ",";

  // 摇杆模拟值
  dataPacket += "LX:" + String(ps2x.Analog(PSS_LX)) + ",";
  dataPacket += "LY:" + String(ps2x.Analog(PSS_LY)) + ",";
  dataPacket += "RX:" + String(ps2x.Analog(PSS_RX)) + ",";
  dataPacket += "RY:" + String(ps2x.Analog(PSS_RY));

  // 如果数据未发生变化，则不发送
  if (dataPacket != lastDataPacket) {
    Serial.println(dataPacket);  // 发送数据包
    lastDataPacket = dataPacket;  // 更新存储的上一次数据包
  }

  delay(50);  // 延迟，避免过高的发送频率
}