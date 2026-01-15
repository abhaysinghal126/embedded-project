#include <Wire.h>
#include <LiquidCrystal.h>

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
float compassOffset = 0; // automatically calculated in setup

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
void driveDistanceCM(int cm);
void turnToDegree(float deg);

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
  bool turnRight = diff > 0;

  // Motor speed based on heading error
  if(absDiff > 25) setMotors(turnRight ? 200 : -200, turnRight ? -200 : 200);
  else if(absDiff > 10) setMotors(turnRight ? 120 : -120, turnRight ? -120 : 120);
  else if(absDiff > 4) setMotors(turnRight ? 80 : -80, turnRight ? -80 : 80);
  else if(absDiff > 1.5) setMotors(turnRight ? 50 : -50, turnRight ? -50 : 50);
  else { 
    stopMotors(); 
    return true; 
  }

  return false;
}

// ===================== FACE NORTH =====================
void faceNorth(){
  while(!turnToHeading(0)){ 
    delay(20); // delay for compass stabilization
  }
  stopMotors();
}

// ===================== DRIVE =====================
void driveDistanceCM(int cm){
  if(cm == 0) return;
  int dir = cm > 0 ? 1 : -1;
  setMotors(dir * motorSpeedPWM, dir * motorSpeedPWM);
  delay(abs(cm) * 50); // simple timing, adjust experimentally
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
  int x = analogRead(JOY_X) - 512;  // left/right
  int y = 512 - analogRead(JOY_Y);  // forward/backward
  x = -x; // invert direction if needed

  int leftSpeed  = y + x;
  int rightSpeed = y - x;

  leftSpeed  = map(constrain(leftSpeed, -512, 512), -512, 512, -motorSpeedPWM, motorSpeedPWM);
  rightSpeed = map(constrain(rightSpeed, -512, 512), -512, 512, -motorSpeedPWM, motorSpeedPWM);

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
  else if(msg == "find:North"){
    faceNorth();
  } 
  else if(msg.startsWith("Speed:")){
    int s = msg.substring(6).toInt();
    if(s >= 0 && s <= 255) motorSpeedPWM = s;
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
  lcd.setCursor(0,1);
  lcd.print("Ready");
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

  // Update LCD: heading
  lcd.setCursor(0,1);
  lcd.print("H:");
  lcd.print(getHeading(), 0);
}
