#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>

#define MAX_POINTS 10

// ** 角度插值计算类 **
class AngleToPInterpolator {
  private:
    float angle[MAX_POINTS];
    float p[MAX_POINTS];
    int size;

  public:
    AngleToPInterpolator() : size(0) {}

    bool addPoint(float angleVal, float pVal) {
      if (size >= MAX_POINTS) return false;
      
      int i;
      for (i = size - 1; (i >= 0 && angle[i] > angleVal); i--) {
        angle[i + 1] = angle[i];
        p[i + 1] = p[i];
      }
      
      angle[i + 1] = angleVal;
      p[i + 1] = pVal;
      size++;
      return true;
    }

    float getP(float angleVal) {
      if (size == 0) return 0;
      if (angleVal <= angle[0]) return p[0];
      if (angleVal >= angle[size - 1]) return p[size - 1];

      int i = 0;
      while (i < size - 1 && angle[i + 1] < angleVal) i++;

      float a0 = angle[i], a1 = angle[i + 1];
      float p0 = p[i], p1 = p[i + 1];
      return p0 + (p1 - p0) * (angleVal - a0) / (a1 - a0);
    }
};

// ** 速度映射类 **
class SpeedMapper {
  public:
    static int mapSpeed(float speed, float min_speed = 0.20, float max_speed = 1.5, float min_mapped = 35, float max_mapped = 55) {
      if (speed <= 0.05) return 0;
      speed = min(speed, max_speed);
      float normalized_speed = (speed - min_speed) / (max_speed - min_speed);
      float sigmoid_speed = 1.0 / (1.0 + exp(-10.0 * (normalized_speed - 0.5)));
      return (int)(sigmoid_speed * (max_mapped - min_mapped) + min_mapped);
    }
};

// ** 无线通信类 **
class RF24Communicator {
  private:
    RF24 radio;
    byte tx_buf[32];

  public:
    RF24Communicator(int cePin, int csnPin) : radio(cePin, csnPin) {}

    bool setup(const byte* txAddress) {
      if (!radio.begin()) return false;
      radio.setPayloadSize(32);
      radio.setChannel(0);
      radio.setCRCLength(RF24_CRC_16);
      radio.setPALevel(RF24_PA_MAX);
      radio.setDataRate(RF24_2MBPS);
      radio.openWritingPipe(txAddress);
      radio.stopListening();
      return true;
    }

    void sendData(byte* data, size_t size) {
      radio.write(data, size);
    }
};

// ** 串口数据解析类 **
class SerialParser {
  public:
    struct ControlCommand {
      float steering_tire_angle;
      float speed;
    };

    static bool parseData(byte* data, ControlCommand& command) {
      if (data[0] != 0x42) return false;
      command.steering_tire_angle = *(float*)(data + 2);
      command.speed = *(float*)(data + 6);
      return true;
    }
};

// ** 控制器类 **
class Controller {
  private:
    AngleToPInterpolator interp;
    RF24Communicator radio;
    unsigned long lastReceivedTime;
    bool timeoutSent;

  public:
    Controller(int cePin, int csnPin) : radio(cePin, csnPin), lastReceivedTime(0), timeoutSent(false) {}

    void setup() {
      Serial.begin(115200);

      byte TX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      if (!radio.setup(TX_address)) {
        Serial.println("RF24 初始化失败");
        while (1);
      }

      interp.addPoint(0, 0);
      interp.addPoint(2, 8);
      interp.addPoint(5, 15);
      interp.addPoint(8, 20);
      interp.addPoint(14, 50);
      interp.addPoint(15, 60);
      interp.addPoint(20, 85);
      interp.addPoint(25, 100);
      interp.addPoint(30, 120);

      lastReceivedTime = millis();
    }

    void loop() {
      if (Serial.available() > 0) {
        byte data[11];
        Serial.readBytes(data, 11);

        lastReceivedTime = millis();
        timeoutSent = false;

        SerialParser::ControlCommand command;
        if (SerialParser::parseData(data, command)) {
          sendStateMessage(0x01, command.steering_tire_angle, command.speed);
          sendToRF24(command);
        } else {
          sendStateMessage(0x04, 0, 0);
        }
      } else {
        checkTimeout();
      }

      delay(20);
    }

  private:
    void sendToRF24(SerialParser::ControlCommand& command) {
      byte tx_buf[10] = {0x55, 0x7E, 0x01};

      if (command.steering_tire_angle < 0) {
        tx_buf[3] = (byte)(120 - interp.getP(-command.steering_tire_angle));
      } else {
        tx_buf[3] = (byte)(120 + interp.getP(command.steering_tire_angle));
      }

      if (command.speed < 0) {
        tx_buf[5] = (byte)(128 - SpeedMapper::mapSpeed(-command.speed));
      } else {
        tx_buf[5] = (byte)(128 + SpeedMapper::mapSpeed(command.speed));
      }

      tx_buf[7] = 0x7E;
      tx_buf[8] = 0x55;
      radio.sendData(tx_buf, sizeof(tx_buf));
    }

    void sendStateMessage(byte flag, float angle, float speed) {
      Serial.print("状态: ");
      Serial.print(flag);
      Serial.print(" 角度: ");
      Serial.print(angle);
      Serial.print(" 速度: ");
      Serial.println(speed);
    }

    void checkTimeout() {
      if ((millis() - lastReceivedTime > 5000) && !timeoutSent) {
        sendStateMessage(0x05, 0, 0);
        timeoutSent = true;
      }
    }
};

// ** 主程序 **
Controller controller(7, 8);

void setup() {
  controller.setup();
}

void loop() {
  controller.loop();
}