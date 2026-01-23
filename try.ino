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
float compassOffset = 90;

// ================= SPEED =================
int motorSpeedPWM = 200;

// ================= JOYSTICK =================
#define JOY_X A10
#define JOY_Y A9
#define JOY_BTN 19

// ================= CONTROL MODE =================
enum ControlMode {
  MODE_JOYSTICK,
  MODE_ESP
};

ControlMode controlMode = MODE_JOYSTICK;

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
  if(h < 0) return -1;

  h += compassOffset;
  if(h >= 360) h -= 360;
  if(h < 0) h += 360;
  return h;
}

// ================= CALIBRATION =================
void calibrateCompass(){
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(0x00);   // command register
  Wire.write(0xF0);   // enter calibration mode
  Wire.endTransmission();
  lcd.setCursor(0,1);
  lcd.print("Calibrating...");
}

// ================= TURN =================
bool turnToHeading(float target){
  float cur = getHeading();
  if(cur < 0) return false;

  float diff = fmod((target - cur + 540), 360) - 180;
  float ad = abs(diff);
  bool right = diff > 0;

  if(ad > 20)      setMotors(right?180:-180, right?-180:180);
  else if(ad > 5)  setMotors(right?90:-90, right?-90:90);
  else {
    stopMotors();
    return true;
  }
  return false;
}

void turnToDegree(float deg){
  espBusy = true;
  while(!turnToHeading(deg)){
    delay(20);
  }
  stopMotors();
  espBusy = false;
}

void faceNorth(){
  turnToDegree(0);
}

// ================= DRIVE =================
void driveDistanceCM(int cm){
  espBusy = true;
  int dir = cm>0?1:-1;
  setMotors(dir*motorSpeedPWM, dir*motorSpeedPWM);
  delay(abs(cm)*80);
  stopMotors();
  espBusy = false;
}

// ================= JOYSTICK =================
void readJoystick(){
  int rawX = analogRead(JOY_X) - 512;
  int rawY = analogRead(JOY_Y) - 512;

  int x = -rawX;
  int y = -rawY;

  int leftSpeed  = y + x;
  int rightSpeed = y - x;

  leftSpeed  = map(constrain(leftSpeed,-512,512),-512,512,-motorSpeedPWM,motorSpeedPWM);
  rightSpeed = map(constrain(rightSpeed,-512,512),-512,512,-motorSpeedPWM,motorSpeedPWM);

  setMotors(leftSpeed,rightSpeed);
}

// ================= HANDLE COMMAND =================
void handleCommand(String cmd){
  cmd.trim();

  if(cmd.startsWith("Dir:") || cmd.startsWith("Turn:")){
    float deg = cmd.substring(cmd.indexOf(':')+1).toFloat();
    turnToDegree(deg);
  }
  else if(cmd == "find:North"){
    faceNorth();
  }
  else if(cmd.startsWith("Move:")){
    driveDistanceCM(cmd.substring(5).toInt());
  }
  else if(cmd.startsWith("Speed:")){
    motorSpeedPWM = constrain(cmd.substring(6).toInt(),0,255);
  }
  else if(cmd == "Calibrate"){
    calibrateCompass();
  }
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
  if(millis()-lastLCD < 250) return;
  lastLCD = millis();
  lcd.setCursor(0,1);
  lcd.print("H:");
  lcd.print((int)getHeading());
  lcd.print("    ");
}

// ================= MODE SWITCH =================
void checkModeSwitch(){
  static bool lastBtn = HIGH;
  bool curBtn = digitalRead(JOY_BTN);

  if(lastBtn==HIGH && curBtn==LOW){
    controlMode = (controlMode==MODE_JOYSTICK)?MODE_ESP:MODE_JOYSTICK;

    lcd.setCursor(0,0);
    if(controlMode==MODE_JOYSTICK)
      lcd.print("Mode:Joystick ");
    else
      lcd.print("Mode:ESP      ");

    delay(200); // debounce
  }
  lastBtn = curBtn;
}

// ================= SETUP =================
void setup(){
  Serial.begin(9600);
  Serial2.begin(9600);
  Wire.begin();

  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  pinMode(JOY_BTN, INPUT_PULLUP);

  lcd.begin(16,2);
  lcd.print("Mode:Joystick");
}

// ================= LOOP =================
void loop(){
  checkModeSwitch();

  if(controlMode==MODE_JOYSTICK && !espBusy){
    readJoystick();
  }

  if(controlMode==MODE_ESP){
    readSerial(Serial, usbBuffer);
    readSerial(Serial2, espBuffer);
  }

  updateLCD();
}
