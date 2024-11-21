#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <Servo.h> // Include the Servo library for the hobby motor

// GPS Module
SoftwareSerial GPSSerial(8, 7); // GPS Serial (RX=8, TX=7)
Adafruit_GPS GPS(&GPSSerial);

// RF Transceiver
#define RF69_CS 7
#define RF69_INT 2
#define RF69_RST 8
RH_RF69 rf69(RF69_CS, RF69_INT);

// Motor and Servo Pins
#define STEERING_SERVO_PIN 4
#define MOTOR_R_CONTROL_PIN 5
#define MOTOR_L_CONTROL_PIN 6

Servo steeringServo; // Create a Servo object for the hobby motor

// Joystick Data Structure
struct JoystickData {
  uint8_t buttonState; // 0: not pressed, 1: pressed
  int16_t x;           // Joystick X-axis (-512 to 512)
  int16_t y;           // Joystick Y-axis (-512 to 512)
};

// GPS Data Structure
struct GPSData {
  float latitude;
  float longitude;
  bool valid;
};

// System Modes
enum Mode { REMOTE_CONTROL, GPS_CONTROL };
Mode currentMode = REMOTE_CONTROL;

// Timing Variables
unsigned long buttonPressTime = 0;
unsigned long debounceTime = 50;
bool buttonPressed = false;

void setup() {
  Serial.begin(9600); // Debugging
  GPSSerial.begin(9600); // GPS module

  // GPS Initialization
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);
  delay(50);

  // RF Transceiver Initialization
  pinMode(RF69_RST, OUTPUT);
  digitalWrite(RF69_RST, LOW);
  delay(10);
  digitalWrite(RF69_RST, HIGH);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RF69 init failed!");
    while (1);
  }
  if (!rf69.setFrequency(915.0)) {
    Serial.println("RF69 setFrequency failed!");
    while (1);
  }
  Serial.println("RF69 initialized.");

  // Servo Initialization
  steeringServo.attach(STEERING_SERVO_PIN); // Attach the hobby motor to the servo pin
  steeringServo.write(90); // Center the steering servo at 90 degrees (neutral position)

  Serial.println("Setup complete.");
}

void loop() {
  // Get Joystick data
  JoystickData joystickData = getJoystickData();

  // Handle button press logic
  if (joystickData.buttonState == 1) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressTime = millis();
    }
  } else {
    if (buttonPressed) {
      unsigned long pressDuration = millis() - buttonPressTime;

      if (pressDuration >= 2000) {
        // Long press: Stop
        Serial.println("STOP");
        analogWrite(MOTOR_R_CONTROL_PIN, 0);
        analogWrite(MOTOR_L_CONTROL_PIN, 0);
      } else if (pressDuration >= debounceTime) {
        // Short press: Switch mode
        switchMode();
      }

      buttonPressed = false;
    }
  }

  // Perform actions based on the current mode
  if (currentMode == REMOTE_CONTROL) {
    controlWithJoystick(joystickData);
  } else if (currentMode == GPS_CONTROL) {
    controlMotorsWithGPS();
  }

  // Continuously send GPS data via RF
  sendGPSData();

  delay(100); // Adjust as needed
}

// Control motors and steering based on joystick input
void controlWithJoystick(JoystickData joystickData) {
  // Map joystick Y-axis to speed
  int baseSpeed = map(joystickData.y, -512, 512, -255, 255);

  // Map joystick X-axis to steering angle
  int steeringAngle = map(joystickData.x, -512, 512, 45, 135); // Steering angle range: 45° to 135°

  // Set steering angle
  steeringServo.write(steeringAngle);

  // Set motor speeds
  int motorSpeed = abs(baseSpeed);
  if (baseSpeed > 0) { // Forward
    analogWrite(MOTOR_L_CONTROL_PIN, motorSpeed);
    analogWrite(MOTOR_R_CONTROL_PIN, motorSpeed);
  } else if (baseSpeed < 0) { // Reverse
    analogWrite(MOTOR_L_CONTROL_PIN, motorSpeed);
    analogWrite(MOTOR_R_CONTROL_PIN, motorSpeed);
  } else { // Stop
    analogWrite(MOTOR_L_CONTROL_PIN, 0);
    analogWrite(MOTOR_R_CONTROL_PIN, 0);
  }

  Serial.print("Joystick Control - Steering Angle: ");
  Serial.print(steeringAngle);
  Serial.print(", Motor Speed: ");
  Serial.println(baseSpeed);
}

// Control motors based on GPS data
void controlMotorsWithGPS() {
  GPSData gpsData = getGPSData();
  if (!gpsData.valid) {
    Serial.println("GPS Control - No valid GPS data.");
    return;
  }

  // For simplicity, generate speed and turning data based on a mock target
  float targetLatitude = 40.0; // Example target latitude
  float targetLongitude = -75.0; // Example target longitude

  float deltaX = targetLongitude - gpsData.longitude;
  float deltaY = targetLatitude - gpsData.latitude;

  // Map deltaX and deltaY to speed and steering
  int baseSpeed = constrain(map(deltaY, -0.1, 0.1, -255, 255), -255, 255);
  int steeringAngle = constrain(map(deltaX, -0.1, 0.1, 45, 135), 45, 135); // Steering range: 45° to 135°

  // Set steering angle
  steeringServo.write(steeringAngle);

  // Set motor speeds
  analogWrite(MOTOR_L_CONTROL_PIN, abs(baseSpeed));
  analogWrite(MOTOR_R_CONTROL_PIN, abs(baseSpeed));

  Serial.print("GPS Control - Steering Angle: ");
  Serial.print(steeringAngle);
  Serial.print(", Motor Speed: ");
  Serial.println(baseSpeed);
}

// Function to switch between modes
void switchMode() {
  if (currentMode == REMOTE_CONTROL) {
    currentMode = GPS_CONTROL;
    Serial.println("Switched to GPS Control mode.");
  } else {
    currentMode = REMOTE_CONTROL;
    Serial.println("Switched to Remote Control mode.");
  }
}

// Function to convert GPS to decimal
float convertGPSToDecimal(float coordinate, char hemisphere) {
  int degrees = (int)(coordinate / 100);
  float minutes = coordinate - degrees * 100;
  float decimal = degrees + minutes / 60.0;
  if (hemisphere == 'S' || hemisphere == 'W') decimal = -decimal;
  return decimal;
}

// Function to get GPS data
GPSData getGPSData() {
  GPSData data = {0.0, 0.0, false};
  GPS.read();
  if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA()) && GPS.fix) {
    data.latitude = convertGPSToDecimal(GPS.latitude, GPS.lat);
    data.longitude = convertGPSToDecimal(GPS.longitude, GPS.lon);
    data.valid = true;
  }
  return data;
}

// Function to get Joystick data
JoystickData getJoystickData() {
  JoystickData data = {0, 0, 0};
  if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len) && len == sizeof(JoystickData)) {
      memcpy(&data, buf, sizeof(JoystickData));
    }
  }
  return data;
}

// Function to send GPS data via RF
void sendGPSData() {
  GPSData gpsData = getGPSData();
  if (!gpsData.valid) return;

  // Prepare GPS data for transmission
  float dataToSend[2] = {gpsData.latitude, gpsData.longitude};

  // Transmit GPS data
  rf69.send((uint8_t *)dataToSend, sizeof(dataToSend));
  rf69.waitPacketSent();

  Serial.print("Sent GPS Data - Latitude: ");
  Serial.print(gpsData.latitude, 6);
  Serial.print(", Longitude: ");
  Serial.println(gpsData.longitude, 6);
}
