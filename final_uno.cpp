#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <RH_RF69.h> // RF模块库

// OLED显示配置
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// RF模块配置
#define RFM69_CS 5
#define RFM69_INT 2
#define RFM69_RST 6
#define FREQUENCY 915.0 // 根据小车配置选择频率
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// 操纵杆配置
int joystickX = A2;
int joystickY = A3;
int joystickButton = 10;

void setup() {
  // 初始化串口
  Serial.begin(9600);

  // 初始化OLED
  if (!display.begin(SSD1306_I2C_ADDRESS, 0x3C)) {
    Serial.println("OLED初始化失败");
    for (;;); // 停止运行
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // 初始化RF模块
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);

  if (!rf69.init()) {
    display.println("RF模块初始化失败");
    display.display();
    while (1);
  }
  rf69.setFrequency(FREQUENCY);
  rf69.setTxPower(20, true);

  // 初始化操纵杆按钮
  pinMode(joystickButton, INPUT_PULLUP);

  display.println("系统初始化完成");
  display.display();
  delay(1000);
}

void loop() {
  // 读取操纵杆值
  int xValue = analogRead(joystickX);
  int yValue = analogRead(joystickY);
  bool buttonPressed = !digitalRead(joystickButton);

  // 映射操纵杆值到-100到100
  int xMapped = map(xValue, 0, 1023, -100, 100);
  int yMapped = map(yValue, 0, 1023, -100, 100);

  // 显示操纵杆状态
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("X: ");
  display.println(xMapped);
  display.print("Y: ");
  display.println(yMapped);
  display.print("Button: ");
  display.println(buttonPressed ? "Pressed" : "Released");
  display.display();

  // 发送数据到小车
  int data[] = {xMapped, yMapped, buttonPressed};
  rf69.send((uint8_t *)data, sizeof(data));
  rf69.waitPacketSent();

  delay(100); // 延迟以防止过快发送
}
