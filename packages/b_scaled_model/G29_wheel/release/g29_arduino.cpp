#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>
// #include <queue> // Not used, can be removed

#define MAX_POINTS 10

// 定义时间戳相关的常量和变量
#define TIME_SYNC_HEADER 0x43
#define ACK_HEADER 0x44
#define ACK_RECEIVED 0x10
#define ACK_SENT 0x11
#define SYNC_REQUEST_TIMEOUT 10000  // 10秒超时，用于响应同步请求
#define SYNC_FAILURE_HEADER 0x45    // 同步失败消息的新标头

// 定义喇叭相关常量
#define TRUMPET_ACTIVE 0x01
#define TRUMPET_INACTIVE 0x00

// 时间戳相关变量
unsigned long localTime = 0;
bool timeIsSynced = false;
bool syncRequestReceived = false;   // 标志，用于跟踪是否收到同步请求但尚未响应
unsigned long lastSyncRequestTime = 0; // 上次同步请求的时间

// 喇叭状态变量
bool trumpetActive = false;  // 喇叭激活状态

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

AngleToPInterpolator interp;

const int CE_PIN = 7;
const int CSN_PIN = 8;

const byte RX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const byte TX_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

RF24 radio(CE_PIN, CSN_PIN);

struct ControlCommand {
  float steering_tire_angle;
  float speed;
};

ControlCommand control_command; // Not explicitly used for storing incoming, consider removal if only used for error state
byte tx_buf[32];

unsigned long lastReceivedTime = 0;
bool timeoutSent = false;

int map_speed_sigmoid(float speed, float min_speed = 0.20, float max_speed = 1.3, float min_mapped = 5, float max_mapped = 40);
void sendStateMessage(byte flag, float steering_tire_angle, float speed); // Forward declaration
byte calculateChecksumInterval(byte array[], int head, int tail); // Forward declaration
void sendAckMessage(byte ackType, unsigned long pcTimestamp); // Forward declaration
void sendDecodingStatus(byte flag, float orig_steering, float orig_speed, byte mapped_steering, byte mapped_speed); // Forward declaration


void setup() {
  Serial.begin(115200);
  
  if (!radio.begin()) {
    // Try to send state message even if radio fails, but Serial must be working
    // control_command is not initialized here, sending 0,0
    sendStateMessage(0x00, 0.0f, 0.0f); 
    while (1) {} 
  }

  radio.setPayloadSize(32);
  radio.setChannel(0);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.openWritingPipe(TX_address);
  radio.stopListening();

  trumpetActive = false;

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
  localTime = millis(); // Initialize localTime
}

void loop() {
  localTime = millis(); // Update local time at the beginning of the loop

  if (syncRequestReceived && (localTime - lastSyncRequestTime > SYNC_REQUEST_TIMEOUT)) {
    sendSyncFailure(0x01); // 1 = 超时
    syncRequestReceived = false; // Reset the flag
    // OPTIMIZATION: Consider if this state message is critical or adds too much serial traffic
    // sendStateMessage(0x06, 0, 0); 
  }
  
  if (Serial.available() > 0) {
    byte header = Serial.peek();
    
    if (header == TIME_SYNC_HEADER && Serial.available() >= 6) { // Packet: Header(1) + PCTimestamp(4) + Checksum(1) = 6
      processSyncPacket();
    } else if (header == 0x42 && Serial.available() >= 16) { // Packet: Header(1) + Type(1) + PCTimestamp(4) + Angle(4) + Speed(4) + Trumpet(1) + Checksum(1) = 16
      processControlPacket();
    }
    // OPTIONAL: Add handling for incomplete/unknown packets to prevent them from blocking peek() indefinitely
    // else if (Serial.available() > SOME_MAX_EXPECTED_PACKET_SIZE_OR_TIMEOUT_ON_PEEK) {
    //   while(Serial.available()) Serial.read(); // Clear buffer if unknown data accumulates
    // }
  } else { 
    if (((unsigned long)(localTime - lastReceivedTime) > 5000) && !timeoutSent) {
      sendStateMessage(0x05, 0, 0); // Send timeout message
      timeoutSent = true;
    }
  }
  
  // OPTIMIZATION: Removed delay(10); The loop should run as fast as possible.
  // Any necessary timing should be handled with millis() for non-blocking delays.
}

void processSyncPacket() {
  byte data[6]; // Header(1) + PCTimestamp(4) + Checksum(1)
  Serial.readBytes(data, 6);
  
  // Basic checksum for sync packet (XOR all bytes including header and checksum byte itself should be 0 if checksum is XOR of preceding bytes)
  byte calculated_checksum = 0;
  for(int i=0; i < 5; i++) { // Checksum over Header + PC Timestamp
    calculated_checksum ^= data[i];
  }

  if (data[0] == TIME_SYNC_HEADER && calculated_checksum == data[5]) {
    syncRequestReceived = true;
    lastSyncRequestTime = millis(); // Use current localTime
    
    unsigned long pcTimestamp = 0;
    pcTimestamp |= (unsigned long)data[1] << 0;
    pcTimestamp |= (unsigned long)data[2] << 8;
    pcTimestamp |= (unsigned long)data[3] << 16;
    pcTimestamp |= (unsigned long)data[4] << 24;
    
    timeIsSynced = true; // Set this flag, though its usage is not apparent in the provided snippet
    
    if (sendSyncAck(pcTimestamp)) {
      syncRequestReceived = false; // Successfully sent ACK, clear the flag
    } else {
      // ACK send failed (e.g., TX buffer full). syncRequestReceived remains true.
      // This will eventually lead to SYNC_REQUEST_TIMEOUT if buffer doesn't clear.
      // Consider sending a specific error code if this state is problematic.
      // sendStateMessage(0x09, 0, 0); // Example: 0x09 for SYNC_ACK_TX_FAIL
    }
  } else {
    // Checksum failed or wrong header
    // Consider sending an error or logging
  }
}

bool sendSyncAck(unsigned long pcTimestamp) {
  byte response[10]; 
  response[0] = TIME_SYNC_HEADER; // Echoing the header
  response[1] = 0x01; // ACK Type
  
  response[2] = (byte)(pcTimestamp & 0xFF);
  response[3] = (byte)((pcTimestamp >> 8) & 0xFF);
  response[4] = (byte)((pcTimestamp >> 16) & 0xFF);
  response[5] = (byte)((pcTimestamp >> 24) & 0xFF);
  
  unsigned long currentLocalTime = millis(); // Get current time for the ACK
  response[6] = (byte)(currentLocalTime & 0xFF);
  response[7] = (byte)((currentLocalTime >> 8) & 0xFF);
  response[8] = (byte)((currentLocalTime >> 16) & 0xFF);
  response[9] = (byte)((currentLocalTime >> 24) & 0xFF);
  
  byte checksum = 0;
  for (int i = 0; i < 10; i++) {
    checksum ^= response[i];
  }
  
  if (Serial.availableForWrite() >= 11) { // 10 bytes for response + 1 for checksum
    Serial.write(response, 10);
    Serial.write(checksum);
    return true;
  } else {
    return false; // TX buffer likely full
  }
}

void sendSyncFailure(byte reason) {
  byte response[6]; // Header(1) + Reason(1) + ArduinoTime(4)
  response[0] = SYNC_FAILURE_HEADER;
  response[1] = reason;

  unsigned long current_time = millis();
  response[2] = (byte)(current_time & 0xFF);
  response[3] = (byte)((current_time >> 8) & 0xFF);
  response[4] = (byte)((current_time >> 16) & 0xFF);
  response[5] = (byte)((current_time >> 24) & 0xFF);
  
  byte checksum = 0;
  for (int i = 0; i < 6; i++) {
    checksum ^= response[i];
  }
  
  if (Serial.availableForWrite() >= 7) { // 6 bytes for response + 1 for checksum
    Serial.write(response, 6);
    Serial.write(checksum);
  }
}

void processControlPacket() {
  byte data[16]; // Header(1)+Type(1)+PCTimestamp(4)+Angle(4)+Speed(4)+Trumpet(1)+Checksum(1)
  Serial.readBytes(data, 16);

  lastReceivedTime = millis(); // Reset timeout counter
  timeoutSent = false;

  byte calculated_checksum = 0;
  for(int i = 1; i < 15; i++) { // Checksum over Type, PCTimestamp, Angle, Speed, Trumpet
      calculated_checksum ^= data[i];
  }

  // Assuming data[0] is the overall packet header (e.g. 0x42)
  // and data[1] is the message type (e.g. 0x01 for command, 0x01 for NRF init in Python - this is confusing, should be distinct)
  // Python sends 0x42 as header, then MSG_TYPE_COMMAND (0x01) or MSG_TYPE_NRF_INIT (0x01)
  // Let's assume data[0] is always 0x42 for control packets from Python.
  // The checksum in Python `calculate_checksum_with_timestamp_and_trumpet` is over:
  // struct.pack('<BIffB', msg_type, timestamp, steering_tire_angle, speed, trumpet_active)
  // This means Python's checksum is over data[1] to data[14] (inclusive of data[14] which is trumpet state)
  // So Arduino should verify data[15] against checksum of data[1]...data[14]

  byte python_checksum_payload_length = 1 + 4 + 4 + 4 + 1; // Type(1) + Timestamp(4) + Angle(4) + Speed(4) + Trumpet(1) = 14 bytes
  calculated_checksum = 0;
  for(int i = 0; i < python_checksum_payload_length; i++) {
    calculated_checksum ^= data[1+i]; // data[1] is type, data[2-5] is timestamp, etc. data[14] is trumpet
  }


  if (data[0] == 0x42 && calculated_checksum == data[15]) { // data[15] is the checksum byte
    unsigned long pcTimestamp = 0;
    pcTimestamp |= (unsigned long)data[2] << 0;  // Timestamp starts at index 2 (after 0x42 header and 0x01 type)
    pcTimestamp |= (unsigned long)data[3] << 8;
    pcTimestamp |= (unsigned long)data[4] << 16;
    pcTimestamp |= (unsigned long)data[5] << 24;
    
    float steering_tire_angle = 0.0f;
    float speed = 0.0f;

    memcpy(&steering_tire_angle, &data[6], sizeof(float)); // Angle starts at index 6
    memcpy(&speed, &data[10], sizeof(float));              // Speed starts at index 10
    
    bool new_trumpet_state = (data[14] == TRUMPET_ACTIVE); // Trumpet state at index 14
    trumpetActive = new_trumpet_state;
    
    byte mapped_steering;
    if (steering_tire_angle < 0) {
        mapped_steering = (byte)(120 - interp.getP(-steering_tire_angle));
    } else {
        mapped_steering = (byte)(120 + interp.getP(steering_tire_angle));
    }

    byte mapped_speed;
    if (speed < 0) {
        mapped_speed = (byte)(128 - map_speed_sigmoid(-speed));
    } else {
        mapped_speed = (byte)(128 + map_speed_sigmoid(speed));
    }

    // OPTIMIZATION: Reduced serial traffic. ACK_RECEIVED is helpful for RTT but adds traffic.
    // sendAckMessage(ACK_RECEIVED, pcTimestamp); 

    // OPTIMIZATION: Decoding status is verbose. Only enable for debugging if necessary.
    // sendDecodingStatus(0x01, steering_tire_angle, speed, mapped_steering, mapped_speed);
    
    set_txbuff(mapped_steering, mapped_speed); // Includes trumpetActive

    bool ok = radio.write(&tx_buf, sizeof(tx_buf));

    if (ok) {
        sendAckMessage(ACK_SENT, pcTimestamp); // Confirm radio transmission
        // OPTIMIZATION: This state message might be redundant if ACK_SENT is sufficient
        // or could be made more concise.
        sendStateMessage(0x02, (float)mapped_steering, (float)mapped_speed); 
    } else {
      // Send NRF radio send error state
      // control_command fields are not updated with current command, using 0,0 for simplicity
      sendStateMessage(0x03, 0.0f, 0.0f); 
    }
  } else {
    // Checksum error or wrong header
    // control_command fields are not updated, using 0,0 for simplicity
    sendStateMessage(0x04, 0.0f, 0.0f); 
  }
}

void sendAckMessage(byte ackType, unsigned long pcTimestamp) {
  byte ackData[11]; // Header(1) + AckType(1) + PCTimestamp(4) + ArduinoTime(4) + Checksum(1)
  ackData[0] = ACK_HEADER;
  ackData[1] = ackType;
  
  ackData[2] = (byte)(pcTimestamp & 0xFF);
  ackData[3] = (byte)((pcTimestamp >> 8) & 0xFF);
  ackData[4] = (byte)((pcTimestamp >> 16) & 0xFF);
  ackData[5] = (byte)((pcTimestamp >> 24) & 0xFF);
  
  unsigned long arduinoTime = millis();
  ackData[6] = (byte)(arduinoTime & 0xFF);
  ackData[7] = (byte)((arduinoTime >> 8) & 0xFF);
  ackData[8] = (byte)((arduinoTime >> 16) & 0xFF);
  ackData[9] = (byte)((arduinoTime >> 24) & 0xFF);
  
  byte checksum = 0;
  for (int i = 0; i < 10; i++) { // Checksum over first 10 bytes
    checksum ^= ackData[i];
  }
  ackData[10] = checksum;
  
  if (Serial.availableForWrite() >= 11) {
    Serial.write(ackData, 11);
  }
}

// This function was declared but not defined in the original snippet.
// It's also not used after optimizations. Keeping for completeness if re-enabled.
bool verifyChecksum(byte* data, int len) { // Added len parameter
  if (len < 2) return false; // Must have at least one data byte and checksum
  byte checksum = 0;
  for (int i = 0; i < len - 1; i++) { // Iterate up to the byte before checksum
    checksum ^= data[i];
  }
  return checksum == data[len - 1];
}

void sendStateMessage(byte flag, float val1, float val2) { // Renamed params for clarity
  byte data[11]; // Header(1) + Flag(1) + Float1(4) + Float2(4) + Checksum(1)
  data[0] = 0x42; // This is the same header as control commands from PC, might be confusing. Consider a different header for Arduino-to-PC status.
  data[1] = flag;
  
  memcpy(&data[2], &val1, sizeof(float));
  memcpy(&data[6], &val2, sizeof(float));
  
  // Checksum is over Flag, Float1, Float2 (bytes 1 to 9)
  data[10] = calculateChecksumInterval(data, 1, 9); 
  
  if (Serial.availableForWrite() >= 11) {
    Serial.write(data, 11);
  }
}

byte calculateChecksumInterval(byte array[], int head, int tail) {
  byte checksum = 0;
  for (int i = head; i <= tail; i++) {
    checksum ^= array[i];
  }
  return checksum;
}

int map_speed_sigmoid(float speed, float min_speed, float max_speed, float min_mapped, float max_mapped) {
  if (speed <= 0.05 && speed >= -0.05) { // Consider deadband for very low speeds
    return 0;
  }
  // Take absolute speed for mapping, sign is handled before calling this
  float abs_speed = fabs(speed);
  abs_speed = min(abs_speed, max_speed);

  if (abs_speed <= min_speed) return (int)min_mapped; // Or some small starting value if min_mapped is not 0

  float normalized_speed = (abs_speed - min_speed) / (max_speed - min_speed);
  float sigmoid_speed = 1.0 / (1.0 + exp(-10.0 * (normalized_speed - 0.5)));
  float mapped_val = sigmoid_speed * (max_mapped - min_mapped) + min_mapped;
  return (int)mapped_val;
}

void set_txbuff(byte mapped_steering, byte mapped_speed) {
    tx_buf[0] = 0x55;
    tx_buf[1] = 0x7E;
    tx_buf[2] = 0x01; 
    tx_buf[3] = mapped_steering;
    tx_buf[4] = trumpetActive ? TRUMPET_ACTIVE : TRUMPET_INACTIVE;
    tx_buf[5] = mapped_speed;
    tx_buf[6] = 0x00; // acceleration
    tx_buf[7] = calculateChecksumInterval(tx_buf, 2, 6); // Checksum from ST to acceleration
    tx_buf[8] = 0x7E;
    tx_buf[9] = 0x55;
    // Bytes 10-31 are not explicitly set, will be whatever was last in memory or 0 if globally declared.
    // radio.setPayloadSize(32) means 32 bytes will be sent.
    // It's good practice to clear or set the rest of the buffer if their content matters.
    // For now, assuming only first 10 bytes are relevant for the receiver.
    for (int i = 10; i < 32; i++) {
        tx_buf[i] = 0x00; // Clear rest of the buffer
    }
}

void sendDecodingStatus(byte flag, float orig_steering, float orig_speed, byte mapped_steering, byte mapped_speed) {
  // This function is verbose and was commented out for optimization.
  // If re-enabled, ensure Serial.availableForWrite() is checked.
  byte data[16];
  data[0] = 0x46; 
  data[1] = flag;
  
  memcpy(&data[2], &orig_steering, sizeof(float));
  memcpy(&data[6], &orig_speed, sizeof(float));
  
  data[10] = mapped_steering;
  data[11] = mapped_speed;
  data[12] = trumpetActive ? TRUMPET_ACTIVE : TRUMPET_INACTIVE;
  data[13] = 0x00; // Reserved
  
  byte checksum = 0;
  for (int i = 1; i <= 13; i++) { // Checksum from flag to reserved byte
    checksum ^= data[i];
  }
  data[14] = checksum;
  data[15] = 0xAA; // End marker
  
  if (Serial.availableForWrite() >= 16) {
    Serial.write(data, 16);
  }
}