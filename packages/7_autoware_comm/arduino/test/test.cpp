#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>
#include "printf.h"
#define MAX_POINTS 10


class AngleToPInterpolator {
  private:
    float angle[MAX_POINTS];
    float p[MAX_POINTS];
    int size;

  public:
    AngleToPInterpolator() : size(0) {}

    // 添加数据点
    bool addPoint(float angleVal, float pVal) {
      if (size >= MAX_POINTS) return false;
      
      // 找到正确的插入位置
      int i;
      for (i = size - 1; (i >= 0 && angle[i] > angleVal); i--) {
        angle[i + 1] = angle[i];
        p[i + 1] = p[i];
      }
      
      // 插入新点
      angle[i + 1] = angleVal;
      p[i + 1] = pVal;
      size++;
      return true;
    }

    // 执行插值，输入角度a，获得p值
    float getP(float angleVal) {
      // 边界检查
      if (size == 0) return 0;
      if (angleVal <= angle[0]) return p[0];
      if (angleVal >= angle[size - 1]) return p[size - 1];

      // 找到正确的区间
      int i = 0;
      while (i < size - 1 && angle[i + 1] < angleVal) i++;

      // 线性插值
      float a0 = angle[i], a1 = angle[i + 1];
      float p0 = p[i], p1 = p[i + 1];
      return p0 + (p1 - p0) * (angleVal - a0) / (a1 - a0);
    }
};

// 全局插值器实例
AngleToPInterpolator interp;

//this is for uno
// 定义 CE 和 CSN 引脚
const int CE_PIN = 7;
const int CSN_PIN = 8;

// 定义spI通道地址
const byte RX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//末位空置 
const byte TX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


// 创建 RF24 对象
RF24 radio(CE_PIN, CSN_PIN);

// 定义 ControlCommand 结构体
struct ControlCommand {
  float steering_tire_angle;//转向角度
  float speed;//速度
};
//声明 ControlCommand 结构体
ControlCommand control_command;
//声明 定长数据包
byte tx_buf[32];

void setup() {
  
  Serial.begin(9600);
  //Serial.setTimeout(1000); 
  // printf_begin();//打印byte数据
  //sendStateMessage(0x00,control_command.steering_tire_angle, control_command.speed);
   if (!radio.begin()) {
    //sendStateMessage(0x00,control_command.steering_tire_angle, control_command.speed);
     //Serial.println(F("radio hardware is not responding!"));
     //while (1) {}
   }

  //radio.printPrettyDetails();//print original set of nrf
  //Serial.println("\n \n \n");
  radio.setPayloadSize(32);//设置数据包大小为定长32字节（与芯片最兼容）
  radio.setChannel(0);// channel 0 as 2.4GHZ-----（发射频率=2400MHZ + n_channel）
  radio.setCRCLength(RF24_CRC_16);//16位校验码
  radio.setPALevel(RF24_PA_MAX);//0db
  radio.setDataRate(RF24_2MBPS);//空中速率为2MBPS
  radio.openWritingPipe(TX_address);
  radio.stopListening(); // 停止监听，进入发送模式

  //Serial.println("radio setup succeed as: ");
  //radio.printPrettyDetails();

  interp.addPoint(0, 0);
  interp.addPoint(5, 15);
  interp.addPoint(10, 30);
  interp.addPoint(15, 50);
  interp.addPoint(20, 60);
  interp.addPoint(25, 100);
  interp.addPoint(30, 120);

  //Serial.println("AngleToPInterpolator initialized");

}

void loop() {
  // 验证校验和
  //float steering_tire_angle=0;
  //float speed=0;
  if (Serial.available() >0) {
    // 读取数据
//    sendStateMessage(0x00,control_command.steering_tire_angle, control_command.speed);
    byte data[11];
    Serial.readBytes(data, 11);

    // 验证校验和
    if (data[0] == 0x42 ) {
      // 解析消息

      control_command.steering_tire_angle = *(float*)(data + 2);
      control_command.speed = *(float*)(data + 6);
//      float steering_tire_angle = reverseFloat(data + 2);
//      float speed = reverseFloat(data + 6);
      
//      control_command.steering_tire_angle = (float)steering_tire_angle;
//      control_command.speed = (float)speed;
      sendStateMessage(0x01,control_command.steering_tire_angle, control_command.speed);
      set_txbuff(control_command.steering_tire_angle,control_command.speed);
      
      bool ok = radio.write(&tx_buf, sizeof(tx_buf));

      if (ok) {
        sendStateMessage(0x02,control_command.steering_tire_angle, control_command.speed);
      } else {
        sendStateMessage(0x03,control_command.steering_tire_angle, control_command.speed);
      }
    }
    else
    {
        sendStateMessage(0x04,control_command.steering_tire_angle, control_command.speed);
    }
  }
  else
  {
    sendStateMessage(0x05,control_command.steering_tire_angle, control_command.speed);
    }
  delay(200);

}

bool verifyChecksum(byte* data) {
  byte checksum = 0;
  for (int i = 0; i < 10; i++) {
    checksum ^= data[i];
  }
  return checksum == data[10];
}

void sendStateMessage(byte flag, float steering_tire_angle, float speed) {
  byte data[11];
  data[0] = 0x42;
  data[1] = flag;
  *(float*)(data + 2) = steering_tire_angle;
  *(float*)(data + 6) = speed;
  data[10] = calculateChecksumInterval(data,0,9);
  Serial.write(data, 11);
}

byte calculateChecksumInterval(byte array[], int head, int tail) {
  byte checksum = 0;
  for (int i = head; i <= tail; i++) {
    checksum ^= array[i];
  }
  return checksum;
}

// void printHexArray(byte array[], int length) {
//   for (int i = 0; i < length; i++) {
//     Serial.print(array[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println();
// }
int map_speed_sigmoid(float speed, float min_speed = 0.05, float max_speed = 1.8, float min_mapped = 60, float max_mapped = 128) {
  // 设置速度阈值
  if (speed <= 0.05) {
    return 0;
  }
  speed = min(speed, max_speed);
  // 归一化速度值
  float normalized_speed = (speed - min_speed) / (max_speed - min_speed);
  // 应用 sigmoid 函数
  float sigmoid_speed = 1.0 / (1.0 + exp(-10.0 * (normalized_speed - 0.5)));
  // 映射到目标范围
  float mapped_speed = sigmoid_speed * (max_mapped - min_mapped) + min_mapped;
  return (int)mapped_speed;
}
void set_txbuff(float steering_tire_angle, float speed)
{
 
  //txbuff          0     1     2     3     4     5      6     7    8   9
  //txbuff_default 0x55  0x7e  0x01  0x00 0x00   0x00  0x00  0x?? 0x7e 0x55
  tx_buf[0] = 0x55;//包头帧1
  tx_buf[1] = 0x7E;//包头帧2
  tx_buf[2] = 0x01;//ST 0为非NRF，1为NRF
  if(steering_tire_angle<0)
  {
    tx_buf[3] = (byte)( 120 - interp.getP(-steering_tire_angle));
    // Serial.println(interp.getP(-steering_tire_angle));
  }else{
    tx_buf[3] = (byte)( 120 + interp.getP(steering_tire_angle));
  }
  //byte p = interp.getP(angle);steering_angle,转向角度值，这里强制将float放大100倍（保留小数点后两位，后续在控制阶段还需要进行range).
  tx_buf[4] = 0x00;//steering_angle_velocity,在当前控制模式下未使用角速度控制，所以设为0.
  if(speed<0)
  {
    tx_buf[5] = (byte)( 128 - map_speed_sigmoid(-speed));
    // Serial.println(map_speed_sigmoid(-speed));
  }else{
    tx_buf[5] = (byte)( 128 + map_speed_sigmoid(speed));
  }
  //tx_buf[5] = (byte) map_speed_sigmoid(speed);//speed,前进/后退速度值（负值为后退），这里强制将float放大100倍
  tx_buf[6] = 0x00;//acceleratio,加速度，以阶梯形式逐级提高rc小车的速度基数，每级+2.
  tx_buf[7] = calculateChecksumInterval(tx_buf,2,6);//Checksum;计算包的数据段的检验和（2-6）
  tx_buf[8] = 0x7E;//包尾帧1
  tx_buf[9] = 0x55;//包尾帧2
  
}
// 将 float 转换为 byte，保留两位小数精度
byte floatToByte(float value) {
  int scaledValue = (int)(value);
  return (byte)scaledValue;
}

// 将 byte 转换为 float，还原两位小数精度
float byteToFloat(byte value) {
  int scaledValue = (int)value;
  return (float)scaledValue;
}
//float reverseFloat(const byte* data) {
//    byte reversedData[4];
//    reversedData[0] = data[3];
//    reversedData[1] = data[2];
//    reversedData[2] = data[1];
//    reversedData[3] = data[0];
//    
//    float result;
//    memcpy(&result, reversedData, sizeof(float));
//    return result;
//}