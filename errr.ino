#include <Wire.h>
#include <LiquidCrystal.h>

// ================= LCD =================
LiquidCrystal lcd(23,25,27,29,31,33);

// ================= MOTOR =================
#define MOTOR_FORWARD HIGH
#define MOTOR_BACKWARD LOW
#define MOTOR_L_DIR_PIN 7
#define MOTOR_R_DIR_PIN 8
#define MOTOR_L_PWM_PIN 9
#define MOTOR_R_PWM_PIN 10

// ================= COMPASS =================
#define CMPS14_ADDRESS 0x60
float compassOffset = 0;

// ================= SPEED =================
int motorSpeedPWM = 200;

// ================= JOYSTICK =================
#define JOY_X A10
#define JOY_Y A9
#define JOY_BTN 19

// ================= GLOBAL =================
String usbBuffer = "";
String espBuffer = "";
bool espBusy = false;
unsigned long lastLCD = 0;

// ================= MOTOR CONTROL =================
void setMotors(int l, int r){
  digitalWrite(MOTOR_L_DIR_PIN, l>=0?MOTOR_FORWARD:MOTOR_BACKWARD);
  digitalWrite(MOTOR_R_DIR_PIN, r>=0?MOTOR_FORWARD:MOTOR_BACKWARD);
  analogWrite(MOTOR_L_PWM_PIN, abs(constrain(l,-255,255)));
  analogWrite(MOTOR_R_PWM_PIN, abs(constrain(r,-255,255)));
}

void stopMotors(){
  analogWrite(MOTOR_L_PWM_PIN,0);
  analogWrite(MOTOR_R_PWM_PIN,0);
}

// ================= COMPASS =================
float getRawHeading(){
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(0x02);
  Wire.endTransmission();
  Wire.requestFrom(CMPS14_ADDRESS,2);
  if(Wire.available()<2) return -1;
  return ((Wire.read()<<8)|Wire.read())/10.0;
}

float getHeading(){
  float h = getRawHeading();
  if(h<0) return -1;
  h += compassOffset;
  if(h>=360) h-=360;
  if(h<0) h+=360;
  return h;
}

// ================= TURN =================
bool turnToHeading(float target){
  float cur = getHeading();
  if(cur<0) return false;
  float diff = fmod((target-cur+540),360)-180;
  float ad = abs(diff);
  bool right = diff>0;

  if(ad>20)      setMotors(right?180:-180, right?-180:180);
  else if(ad>5)  setMotors(right?90:-90, right?-90:90);
  else {
    stopMotors();
    return true;
  }
  return false;
}

void turnToDegree(float deg){
  espBusy=true;
  while(!turnToHeading(deg)){
    delay(20);
  }
  stopMotors();
  espBusy=false;
}

void faceNorth(){
  turnToDegree(0);
}

// ================= DRIVE =================
void driveDistanceCM(int cm){
  espBusy=true;
  int dir = cm>0?1:-1;
  setMotors(dir*motorSpeedPWM, dir*motorSpeedPWM);
  delay(abs(cm)*80);
  stopMotors();
  espBusy=false;
}

// ================= JOYSTICK =================
void readJoystick(){
  int rawX = analogRead(JOY_X) - 512;
  int rawY = analogRead(JOY_Y) - 512; // sửa lại 512 - ... trước đây
  int x = -rawX;  // invert nếu cần, quẹo đúng chiều
  int y = -rawY;   // forward/backward

  int leftSpeed  = y + x;
  int rightSpeed = y - x;

  // map -512..512 -> -motorSpeedPWM..motorSpeedPWM
  leftSpeed  = map(constrain(leftSpeed,-512,512),-512,512,-motorSpeedPWM,motorSpeedPWM);
  rightSpeed = map(constrain(rightSpeed,-512,512),-512,512,-motorSpeedPWM,motorSpeedPWM);

  setMotors(leftSpeed,rightSpeed);
}

// ================= HANDLE COMMAND =================
void handleCommand(String cmd){
  cmd.trim();
  espBusy=true;

  if(cmd.startsWith("Dir:") || cmd.startsWith("Turn:")){
    float deg = cmd.substring(cmd.indexOf(':')+1).toFloat();
    turnToDegree(deg);
  }
  else if(cmd=="find:North"){
    faceNorth();
  }
  else if(cmd.startsWith("Move:")){
    driveDistanceCM(cmd.substring(5).toInt());
  }
  else if(cmd.startsWith("Speed:")){
    motorSpeedPWM=constrain(cmd.substring(6).toInt(),0,255);
  }

  espBusy=false;
}

// ================= READ SERIAL =================
void readSerial(Stream &port, String &buffer){
  while(port.available()){
    char c = port.read();
    if(c=='\n' || c=='\r'){
      if(buffer.length()){
        handleCommand(buffer);
        buffer="";
      }
    } else buffer+=c;
  }
}

// ================= LCD UPDATE =================
void updateLCD(){
  if(millis()-lastLCD<250) return;
  lastLCD = millis();
  lcd.setCursor(0,1);
  lcd.print("H:    ");
  lcd.setCursor(2,1);
  lcd.print((int)getHeading());
}

// ================= MODE SWITCH =================
void checkModeSwitch(){
  static bool lastBtn = HIGH;
  bool curBtn = digitalRead(JOY_BTN);
  if(lastBtn==HIGH && curBtn==LOW){
    lcd.setCursor(0,0);
    lcd.print("Mode:Joystick ");
    delay(200); // debounce
  }
  lastBtn = curBtn;
}

// ================= SETUP =================
void setup(){
  Serial.begin(9600);    // USB Monitor
  Serial2.begin(9600);   // ESP8266
  Wire.begin();

  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  pinMode(JOY_BTN, INPUT_PULLUP);

  lcd.begin(16,2);
  lcd.print("Calibrating...");

  compassOffset=-getRawHeading();
  if(compassOffset<0) compassOffset+=360;

  lcd.clear();
  lcd.print("Mode:Joystick");
}

// ================= LOOP =================
void loop(){
  checkModeSwitch();
  readJoystick();                // Joystick luôn chạy
  readSerial(Serial, usbBuffer); // USB luôn đọc
  readSerial(Serial2, espBuffer);// ESP luôn đọc
  updateLCD();
}
