
#include <Wire.h>
#include <VL53L0X.h>

#define DEBUG 1              
#define USE_WIRE_TIMEOUT 0   

const uint8_t ENA = 9, IN1 = 7, IN2 = 8;        
const uint8_t ENB = 6, IN3 = 5, IN4 = 4;        
const uint8_t XS[2] = {2, 3};                   
const uint8_t TAPE_PIN = A0;                    
const uint8_t LED_PIN  = 13;
const uint8_t MPU      = 0x68;                 


const bool     FLIP_LEFT = false, FLIP_RIGHT = false;  
const int8_t   GYRO_SIGN = 1;       
const uint8_t  BASE_SPEED = 65;     
const uint8_t  TURN_MIN = 65, TURN_MAX = 100;   
const int8_t   LEFT_TRIM = 0;       
const int8_t   RIGHT_TRIM = 8;      
const uint16_t MM_PER_S = 250;      
const uint16_t FRONT_STOP = 180;    
const uint16_t SIDE_OPEN = 200;     
const uint16_t SIDE_TARGET = 95;    
const uint16_t CREEP_MM = 100;      
const uint16_t CLEAR_MM = 140;      
const uint16_t STARTUP_STRAIGHT_MM = 200; 
const float    KH = 3.0f;           
const float    KW = 0.08f;          
const float    MAX_TILT = 10.0f;    
const bool     TAPE_ENABLED = true;
const bool     TAPE_DARK_HIGH = true;  
const uint16_t TAPE_THRESH = 340;      
const uint16_t PARK_MIN_MM = 100;      
const uint16_t PARK_FRONT_MM = 50;     
const uint16_t PARK_MAX_MM = 300;      
const uint32_t I2C_HZ = 100000;

#define MS(mm) ((uint32_t)(mm) * 1000UL / MM_PER_S)   


VL53L0X  tof[2];                    
uint16_t dF, dR;                    
float    gBias, heading, target;    
uint32_t lastUs, ignoreUntil, startStraightUntil, moveStartMs;
bool     moved = false;             
uint8_t  openCnt, tapeCnt, frontCnt;


void setMotor(int16_t l, int16_t r) {
  if (FLIP_LEFT)  l = -l;
  if (FLIP_RIGHT) r = -r;
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);
  digitalWrite(IN1, l >= 0); digitalWrite(IN2, l < 0); analogWrite(ENA, abs(l));
  digitalWrite(IN3, r >= 0); digitalWrite(IN4, r < 0); analogWrite(ENB, abs(r));
}

void brake() {                      // IN1=IN2 และ EN เปิด = เบรก
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 255);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 255);
}


int16_t gyroRaw() {
  Wire.beginTransmission(MPU); Wire.write(0x47); Wire.endTransmission(false);
  Wire.requestFrom(MPU, (uint8_t)2);
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

void initGyro() {
  Wire.beginTransmission(MPU); Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
  Wire.beginTransmission(MPU); Wire.write(0x1B); Wire.write(0); Wire.endTransmission();   
  Wire.beginTransmission(MPU); Wire.write(0x1A); Wire.write(3); Wire.endTransmission();  
  delay(200);รอให้                              
  for (uint8_t t = 0; t < 5; t++) {        
    long sum = 0;
    int16_t mn = 32767, mx = -32768;
    for (uint16_t i = 0; i < 1000; i++) {
      int16_t g = gyroRaw();
      sum += g;
      if (g < mn) mn = g;
      if (g > mx) mx = g;
      delay(3);
    }
    gBias = (float)sum / 1000.0f;
    if ((mx - mn) < 260) break;            
  }
}

void updateGyro() {
  uint32_t now = micros();
  float dt = (now - lastUs) * 1e-6f;
  lastUs = now;
  float gz = (gyroRaw() - gBias) * (GYRO_SIGN / 131.0f);   
  if (fabs(gz) > 0.3f) heading += gz * dt;
}


void fail(uint8_t n) {              
  brake();
  for (;;) {
    for (uint8_t k = 0; k < n; k++) {
      digitalWrite(LED_PIN, HIGH); delay(200);
      digitalWrite(LED_PIN, LOW);  delay(200);
    }
    delay(1000);
  }
}

void initToF() {
  for (uint8_t i = 0; i < 2; i++) { pinMode(XS[i], OUTPUT); digitalWrite(XS[i], LOW); }
  delay(50);
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(XS[i], INPUT); delay(50);          
    tof[i].setTimeout(100);
    if (!tof[i].init()) fail(i + 1);
    tof[i].setAddress(0x30 + i);               
    tof[i].setMeasurementTimingBudget(20000);  
    tof[i].startContinuous();                  
  }
}


uint16_t rd(uint8_t i, uint16_t bad) {
  uint16_t d = tof[i].readRangeContinuousMillimeters();
  return (tof[i].timeoutOccurred() || d > 1200) ? bad : d;
}

void sense() {                     
  updateGyro();
  dF = rd(0, 0);                    
  dR = rd(1, 1200);                 
  updateGyro();
#if DEBUG
  Serial.print(F("F=")); Serial.print(dF);
  Serial.print(F(" R=")); Serial.print(dR);
  Serial.print(F(" H=")); Serial.println(heading);
#endif
}

bool tapeSeen() {
  if (!TAPE_ENABLED) return false;
  int v = analogRead(TAPE_PIN);
  bool on = TAPE_DARK_HIGH ? (v > (int)TAPE_THRESH) : (v < (int)TAPE_THRESH);
  tapeCnt = on ? min(tapeCnt + 1, 255) : 0;
  return tapeCnt >= 3;              



void drive() {
  
  if (!moved) { moved = true; moveStartMs = millis(); }
  uint32_t rampMs = millis() - moveStartMs;
  int16_t speedNow = (rampMs < 600)
      ? 52 + (int16_t)((BASE_SPEED - 52) * rampMs / 600UL)
      : BASE_SPEED;

  float err = 0;
  if (dR < SIDE_OPEN) err = 2.0f * ((float)dR - (float)SIDE_TARGET);   
  float tilt = constrain(KW * err, -MAX_TILT, MAX_TILT);
  float corr = KH * (heading - (target - tilt));            
  
  int16_t leftSpeed  = speedNow + (int16_t)corr + LEFT_TRIM;
  int16_t rightSpeed = speedNow - (int16_t)corr + RIGHT_TRIM;
  setMotor(leftSpeed, rightSpeed);
}

void turnBy(int16_t angle) {
  brake(); delay(60);
  target += angle;                  
  for (uint8_t k = 0; k < 2; k++) { 
    uint32_t t0 = millis();
    while (millis() - t0 < 2500) {
      updateGyro();
      float e = target - heading;
      if (fabs(e) < 2.0f) break;
      int16_t s = constrain((int16_t)(fabs(e) * 2.5f) + TURN_MIN, TURN_MIN, TURN_MAX);
      if (e > 0) setMotor(-s, s); else setMotor(s, -s);
    }
    brake(); delay(60);
  }
  updateGyro();
}


void park() {
  uint32_t t0 = millis();
  uint32_t tMin = MS(PARK_MIN_MM), tMax = MS(PARK_MAX_MM);
  while (millis() - t0 < tMax) {
    sense();
    if (millis() - t0 > tMin && dF <= PARK_FRONT_MM) break;
    drive();
  }
  brake();
  digitalWrite(LED_PIN, HIGH);
  for (;;) {}
}

void creep(uint16_t mm) {
  uint32_t end = millis() + MS(mm);
  while ((int32_t)(millis() - end) < 0) {
    sense();
    if (tapeSeen()) park();
    if (dF < FRONT_STOP) break;
    drive();
  }
  brake(); delay(60);
}


void setup() {
#if DEBUG
  Serial.begin(115200);
#endif
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  brake();

  Wire.begin();
  Wire.setClock(I2C_HZ);
#if USE_WIRE_TIMEOUT && defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);       
#endif

  digitalWrite(LED_PIN, HIGH);            
  initToF();
  initGyro();
  for (uint8_t k = 0; k < 3; k++) {       
    digitalWrite(LED_PIN, LOW);  delay(150);
    digitalWrite(LED_PIN, HIGH); delay(150);
  }
  digitalWrite(LED_PIN, LOW);
  delay(500);
  heading = 0; target = 0;
  lastUs = micros();
  startStraightUntil = millis() + MS(STARTUP_STRAIGHT_MM); 
}

void loop() {
  sense();
  if (tapeSeen()) park();

  bool ignoring = (int32_t)(millis() - ignoreUntil) < 0;
  bool startupStraight = (int32_t)(millis() - startStraightUntil) < 0;
  if (moved && !startupStraight && !ignoring && dR > SIDE_OPEN) openCnt++;
  else openCnt = 0;
  if (openCnt >= 2) {
    openCnt = 0; frontCnt = 0;
#if DEBUG
    Serial.print(F(">> RIGHT OPEN  R=")); Serial.println(dR);
#endif
    if (moved) creep(CREEP_MM);           
    turnBy(-90);
    ignoreUntil = millis() + MS(CLEAR_MM);
    return;
  }

  
  if (dF > FRONT_STOP) { frontCnt = 0; drive(); return; }

  if (++frontCnt < 2) { brake(); return; }
  frontCnt = 0;
#if DEBUG
  Serial.print(F(">> FRONT BLOCKED  F=")); Serial.println(dF);
#endif
  turnBy(90);
}
