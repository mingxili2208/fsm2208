#define PIN_PWM 10  // D10 引脚
#define DEFAULT_PULSE 1500  // 默认脉冲宽度（μs）
#define MIN_PULSE 1000  // 最小脉冲宽度改为1000μs
#define MAX_PULSE 2000  // 最大脉冲宽度（μs）
// 删除 STEP_SIZE 定义，因为不再需要步长限制

int pulseWidth = DEFAULT_PULSE;  // 当前脉冲宽度
String inputString = "";  // 用于存储串口输入的字符串
boolean stringComplete = false;  // 标记输入是否完成

void setup() {
  // 初始化串口，用于调试和输入
  Serial.begin(115200);
  Serial.println("Starting PWM signal generation...");
  Serial.println("Enter pulse width (1000-2000us) or 's' to reset to 1500us:");

  // 配置 D10 引脚为输出
  pinMode(PIN_PWM, OUTPUT);

  // 关闭全局中断，配置定时器时需要这样做
  cli();

  // --- Timer 1 Configuration ---
  // 1. 设置 Timer 1 的模式为快速 PWM (Fast PWM), TOP 值由 ICR1 寄存器决定
  // WGM13 = 1, WGM12 = 1, WGM11 = 1, WGM10 = 0
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12);

  // 2. 设置预分频器为 8
  // CS12 = 0, CS11 = 1, CS10 = 0
  // 时钟频率 = 16MHz / 8 = 2MHz
  TCCR1B |= (1 << CS11);

  // 3. 设置非反相 PWM 模式 (Non-Inverted PWM)
  // COM1B1 = 1, COM1B0 = 0  (用于 OC1B 引脚，即 D10)
  TCCR1A |= (1 << COM1B1);

  // 4. 设置 TOP 值 (PWM 周期)
  // PWM 频率 = 时钟频率 / TOP
  // 300Hz 频率，所以 TOP = 2000000 / 300 = 6666.666...
  ICR1 = 6667;

  // 5. 设置默认占空比 (脉冲宽度 1500us)
  updatePulseWidth(DEFAULT_PULSE);

  // 重新开启全局中断
  sei();

  Serial.println("PWM signal generation started.");
  Serial.print("Current pulse width: ");
  Serial.print(pulseWidth);
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
    // 检查是否输入了's'，如果是则重置为默认值
    if (inputString.equals("s") || inputString.equals("S")) {
      updatePulseWidth(DEFAULT_PULSE);
      Serial.print("Reset to default pulse width: ");
      Serial.print(pulseWidth);
      Serial.println("us");
    } else {
      // 否则尝试将输入解析为数字
      int newPulse = inputString.toInt();
      
      // 检查输入是否在有效范围内（移除了步长检查）
      if (newPulse >= MIN_PULSE && newPulse <= MAX_PULSE) {
        updatePulseWidth(newPulse);
        Serial.print("Pulse width updated to: ");
        Serial.print(pulseWidth);
        Serial.println("us");
      } else {
        Serial.println("Invalid input! Please enter a value between 1000 and 2000us.");
      }
    }
    
    // 清空输入字符串，准备下一次输入
    inputString = "";
    stringComplete = false;
    Serial.println("Enter pulse width (1000-2000us) or 's' to reset to 1500us:");
  }
}

// 更新脉冲宽度的函数
void updatePulseWidth(int width) {
  pulseWidth = width;
  
  // 计算OCR1B的值
  // PWM周期为6667个时钟周期（对应300Hz@2MHz）
  // 将脉冲宽度(us)映射到OCR1B值
  // 2MHz的时钟，每us是2个时钟周期
  int ocr1bValue = width * 2;
  
  // 更新OCR1B寄存器
  OCR1B = ocr1bValue;
}