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

// 创建RF24Manager实例 (CE引脚=7, CSN引脚=8)
RF24Manager rfManager(7, 8);
byte receivedData[32];

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
  // 读取数据
  uint8_t size;
  if (rfManager.receiveData(receivedData, &size)) {
      Serial.print("Received data (size=");
      Serial.print(size);
      Serial.print("): ");
      
      // 以十六进制打印完整数据
      for (int i = 0; i < size; i++) {
        Serial.print(receivedData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
  }
}