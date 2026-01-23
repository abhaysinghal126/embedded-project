#include <Wire.h>
#include <LiquidCrystal.h>
#include <math.h>

// ===================== LCD =====================
LiquidCrystal lcd(23,25,27,29,31,33); 

// ===================== MOTOR PINS =====================
#define MOTOR_FORWARD HIGH
#define MOTOR_BACKWARD LOW
#define MOTOR_L_DIR_PIN 7
#define MOTOR_R_DIR_PIN 8
#define MOTOR_L_PWM_PIN 9
#define MOTOR_R_PWM_PIN 10

// ===================== COMPASS =====================
#define CMPS14_ADDRESS 0x60
float compassOffset = 90; // automatically calculated in setup

// ===================== MOTOR SPEED =====================
int motorSpeedPWM = 200; // default PWM speed

// ===================== CONTROL MODE =====================
enum ControlMode { JOYSTICK, ESP_CONTROL };
ControlMode currentMode = JOYSTICK;

// ===================== JOYSTICK PINS =====================
#define JOY_X A10
#define JOY_Y A9
#define JOY_BTN 19

// ===================== GLOBAL VARIABLES =====================
String espBuffer = "";
String serialBuffer = "";

// ===================== ENCODER PINS =====================
// Using standard interrupt-capable pins (can be disabled if encoders not connected)
#define ENCODER_L_A 2   // Left encoder channel A (interrupt pin)
#define ENCODER_L_B 3   // Left encoder channel B
#define ENCODER_R_A 20  // Right encoder channel A (interrupt pin)
#define ENCODER_R_B 21  // Right encoder channel B

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long lastEncoderLeft = 0;
long lastEncoderRight = 0;

// Encoder calibration: pulses per cm (adjust based on your wheel/encoder setup)
#define PULSES_PER_CM 10  // Example: 10 pulses per cm, adjust experimentally

// ===================== FUNCTION PROTOTYPES =====================
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();
float getRawHeading();
float getHeading();
bool turnToHeading(float targetHeading);
void faceNorth();
void readJoystick();
void checkModeSwitch();
void handleESPCommand(String msg);
void handleSerialCommand(String msg);
void driveDistanceCM(int cm);
void driveDistanceCMEncoder(int cm);
void turnToDegree(float deg);
String getDirectionString(float heading);
void updateLCD();
void encoderLeftISR();
void encoderRightISR();

// ===================== MOTOR CONTROL =====================
void setMotors(int leftSpeed, int rightSpeed){
  if(leftSpeed >= 0) digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
  else { 
    digitalWrite(MOTOR_L_DIR_PIN, MOTOR_BACKWARD); 
    leftSpeed = -leftSpeed; 
  }
  
  if(rightSpeed >= 0) digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
  else { 
    digitalWrite(MOTOR_R_DIR_PIN, MOTOR_BACKWARD); 
    rightSpeed = -rightSpeed; 
  }

  analogWrite(MOTOR_L_PWM_PIN, constrain(leftSpeed, 0, 255));
  analogWrite(MOTOR_R_PWM_PIN, constrain(rightSpeed, 0, 255));
}

void stopMotors(){
  analogWrite(MOTOR_L_PWM_PIN, 0);
  analogWrite(MOTOR_R_PWM_PIN, 0);
}

// ===================== COMPASS =====================
float getRawHeading(){
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(0x02);
  Wire.endTransmission();
  Wire.requestFrom(CMPS14_ADDRESS, 2);
  if(Wire.available() < 2) return -1;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 10.0;
}

float getHeading(){
  float deg = getRawHeading() + compassOffset;
  if(deg >= 360) deg -= 360;
  if(deg < 0) deg += 360;
  return deg;
}

// ===================== TURN TO HEADING =====================
bool turnToHeading(float target){
  float current = getHeading();
  if(current < 0) return false;
  
  float diff = fmod((target - current + 540.0), 360.0) - 180.0;
  float absDiff = abs(diff);
  
  // Use +/- 1 degree tolerance (e.g., 119-121 for target 120)
  if(absDiff <= 1.0) {
    stopMotors();
    return true;
  }
  
  bool turnRight = diff > 0;

  // Motor speed based on heading error
  if(absDiff > 25) setMotors(turnRight ? 200 : -200, turnRight ? -200 : 200);
  else if(absDiff > 10) setMotors(turnRight ? 120 : -120, turnRight ? -120 : 120);
  else if(absDiff > 4) setMotors(turnRight ? 80 : -80, turnRight ? -80 : 80);
  else setMotors(turnRight ? 50 : -50, turnRight ? -50 : 50);

  return false;
}

// ===================== FACE NORTH =====================
void faceNorth(){
  while(!turnToHeading(0)){ 
    delay(20); // delay for compass stabilization
  }
  stopMotors();
}

// ===================== ENCODER INTERRUPTS =====================
void encoderLeftISR(){
  if(digitalRead(ENCODER_L_B) == HIGH) encoderLeftCount++;
  else encoderLeftCount--;
}

void encoderRightISR(){
  if(digitalRead(ENCODER_R_B) == HIGH) encoderRightCount++;
  else encoderRightCount--;
}

// ===================== DRIVE =====================
void driveDistanceCM(int cm){
  // Try encoder-based first, fallback to timing-based
  driveDistanceCMEncoder(cm);
}

void driveDistanceCMEncoder(int cm){
  if(cm == 0) return;
  
  // Reset encoder counts
  encoderLeftCount = 0;
  encoderRightCount = 0;
  lastEncoderLeft = 0;
  lastEncoderRight = 0;
  
  int dir = cm > 0 ? 1 : -1;
  long targetPulses = abs(cm) * PULSES_PER_CM;
  
  setMotors(dir * motorSpeedPWM, dir * motorSpeedPWM);
  
  // Wait until target distance reached
  long avgPulses = 0;
  unsigned long startTime = millis();
  unsigned long timeout = abs(cm) * 200; // timeout fallback (ms)
  
  while(true){
    avgPulses = (abs(encoderLeftCount) + abs(encoderRightCount)) / 2;
    
    if(avgPulses >= targetPulses){
      break;
    }
    
    // Timeout fallback if encoders not working
    if(millis() - startTime > timeout){
      break;
    }
    
    delay(10);
  }
  
  stopMotors();
}

// ===================== TURN =====================
void turnToDegree(float deg){
  while(!turnToHeading(deg)){ 
    delay(20); 
  }
  stopMotors();
}

// ===================== READ JOYSTICK =====================
void readJoystick(){
  int xRaw = analogRead(JOY_X);
  int yRaw = analogRead(JOY_Y);
  
  // Center values (adjust if your joystick center is different)
  int centerX = 512;
  int centerY = 512;
  int deadZone = 20; // Dead zone to prevent drift
  
  int x = xRaw - centerX;  // left/right
  int y = centerY - yRaw;  // forward/backward
  x = -x; // invert direction if needed
  
  // Calculate distance from center
  float distance = sqrt(x*x + y*y);
  float maxDistance = sqrt(512*512 + 512*512); // Maximum possible distance
  
  // Apply dead zone
  if(abs(x) < deadZone) x = 0;
  if(abs(y) < deadZone) y = 0;
  
  // Scale speed based on distance from center (0 to maxSpeed)
  float speedFactor = constrain(distance / maxDistance, 0.0, 1.0);
  int currentMaxSpeed = (int)(motorSpeedPWM * speedFactor);
  
  // Calculate differential drive speeds
  int leftSpeed  = y + x;
  int rightSpeed = y - x;
  
  // Normalize and scale
  int maxInput = max(abs(leftSpeed), abs(rightSpeed));
  if(maxInput > 0){
    leftSpeed  = map(leftSpeed, -512, 512, -currentMaxSpeed, currentMaxSpeed);
    rightSpeed = map(rightSpeed, -512, 512, -currentMaxSpeed, currentMaxSpeed);
  } else {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  setMotors(leftSpeed, rightSpeed);
}

// ===================== MODE SWITCH =====================
void checkModeSwitch(){
  static bool lastBtn = HIGH;
  bool currBtn = digitalRead(JOY_BTN);
  if(lastBtn == HIGH && currBtn == LOW){
    currentMode = (currentMode == JOYSTICK) ? ESP_CONTROL : JOYSTICK;
    espBuffer = ""; // reset ESP buffer
    lcd.setCursor(0,0);
    lcd.print(currentMode == JOYSTICK ? "Mode: Joystick " : "Mode: ESP      ");
  }
  lastBtn = currBtn;
}

// ===================== GET DIRECTION STRING =====================
String getDirectionString(float heading){
  if(heading >= 337.5 || heading < 22.5) return "N";
  else if(heading >= 22.5 && heading < 67.5) return "NE";
  else if(heading >= 67.5 && heading < 112.5) return "E";
  else if(heading >= 112.5 && heading < 157.5) return "SE";
  else if(heading >= 157.5 && heading < 202.5) return "S";
  else if(heading >= 202.5 && heading < 247.5) return "SW";
  else if(heading >= 247.5 && heading < 292.5) return "W";
  else return "NW";
}

// ===================== UPDATE LCD =====================
void updateLCD(){
  float heading = getHeading();
  String dir = getDirectionString(heading);
  
  lcd.setCursor(0,1);
  lcd.print("H:");
  if(heading < 100) lcd.print(" ");
  if(heading < 10) lcd.print(" ");
  lcd.print(heading, 0);
  lcd.print(" ");
  lcd.print(dir);
  lcd.print("      "); // Clear remaining characters
}

// ===================== HANDLE ESP COMMAND =====================
void handleESPCommand(String msg){
  msg.trim();
  lcd.setCursor(0,1);
  lcd.print(msg + "        "); // clear old characters

  if(msg.startsWith("Move:")){
    int cm = msg.substring(5).toInt();
    driveDistanceCM(cm);
  } 
  else if(msg.startsWith("Turn:")){
    float deg = msg.substring(5).toFloat();
    turnToDegree(deg);
  }
  else if(msg.startsWith("Dir:")){
    float deg = msg.substring(4).toFloat();
    // Normalize to 0-360
    while(deg < 0) deg += 360;
    while(deg >= 360) deg -= 360;
    turnToDegree(deg);
  }
  else if(msg == "find:North"){
    faceNorth();
  } 
  else if(msg.startsWith("Speed:")){
    int s = msg.substring(6).toInt();
    if(s >= 0 && s <= 255) motorSpeedPWM = s;
  }
}

// ===================== HANDLE SERIAL COMMAND =====================
void handleSerialCommand(String msg){
  msg.trim();
  
  if(msg.startsWith("Move:")){
    int cm = msg.substring(5).toInt();
    driveDistanceCM(cm);
  } 
  else if(msg.startsWith("Turn:")){
    float deg = msg.substring(5).toFloat();
    turnToDegree(deg);
  }
  else if(msg.startsWith("Dir:")){
    float deg = msg.substring(4).toFloat();
    // Normalize to 0-360
    while(deg < 0) deg += 360;
    while(deg >= 360) deg -= 360;
    turnToDegree(deg);
  }
  else if(msg == "find:North" || msg == "North"){
    faceNorth();
  } 
  else if(msg.startsWith("Speed:")){
    int s = msg.substring(6).toInt();
    if(s >= 0 && s <= 255) motorSpeedPWM = s;
  }
  else {
    Serial.println("Commands: Move:cm, Turn:deg, Dir:deg, find:North, Speed:0-255");
  }
}

// ===================== SETUP =====================
void setup(){
  Serial.begin(9600);
  Serial2.begin(9600); // RX2 = pin 17

  Wire.begin();

  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  pinMode(JOY_BTN, INPUT_PULLUP);

  // Setup encoders (comment out if encoders not connected)
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderRightISR, RISING);

  stopMotors();

  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Calibrating...");

  // Automatically calculate compass offset
  float initial = getRawHeading();
  compassOffset = 0 - initial;
  if(compassOffset < 0) compassOffset += 360;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Mode: Joystick");
  updateLCD();
  
  Serial.println("Ready! Commands: Move:cm, Turn:deg, Dir:deg, find:North, Speed:0-255");
}

// ===================== LOOP =====================
void loop(){
  checkModeSwitch();

  if(currentMode == JOYSTICK){
    readJoystick();
  }

  if(currentMode == ESP_CONTROL && Serial2.available()){
    char c = Serial2.read();
    if(c == '\n' || c == '\r'){
      if(espBuffer.length() > 0){
        handleESPCommand(espBuffer);
        espBuffer = "";
      }
    } 
    else espBuffer += c;
  }

  // Handle Serial commands (works in both modes)
  if(Serial.available()){
    char c = Serial.read();
    if(c == '\n' || c == '\r'){
      if(serialBuffer.length() > 0){
        handleSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } 
    else serialBuffer += c;
  }

  // Update LCD: heading and direction
  updateLCD();
}
