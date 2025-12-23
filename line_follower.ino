// ================================
// LINE FOLLOWER – TEENSY 4.1
// 16-CH ANALOG SENSOR ARRAY
// Mid-8 = line | Edge-2 = enforce
// DRV8833 READY | COMPETITION SAFE
// ================================
#include <EEPROM.h>
// ---------- TUNING ----------
constexpr float KP = 45.0f;
constexpr float KI = 0.0f;
constexpr float KD = 12.0f;

constexpr int BASE_PWM = 110;
constexpr int MAX_PWM  = 255;

// ---------- SENSOR CONFIG ----------
constexpr int TOTAL_SENSORS = 16;
constexpr int MID_START = 4;
constexpr int MID_END   = 11;
constexpr int EDGE_L    = 3;
constexpr int EDGE_R    = 12;

int sensorPins[TOTAL_SENSORS] = {
  A0,A1,A2,A3,A4,A5,A6,A7,
  A8,A9,A10,A11,A12,A13,A14,A15
};

int threshold[TOTAL_SENSORS];
int analogVal[TOTAL_SENSORS];
bool digitalVal[TOTAL_SENSORS];

// ---------- MOTOR PINS ----------
int LM = 10, LMN = 11;
int RM = 3,  RMN = 9;

// ---------- PID STATE ----------
float integral = 0.0f;
float prevError = 0.0f;
float dFiltered = 0.0f;
unsigned long prevTime = 0;

// ---------- HELPERS ----------
inline int capPWM(int v){
  return v > MAX_PWM ? MAX_PWM : (v < 0 ? 0 : v);
}

// ---------- SENSOR READ ----------
void readSensors(){
  for(int i=0;i<TOTAL_SENSORS;i++){
    analogVal[i] = analogRead(sensorPins[i]);
    digitalVal[i] = analogVal[i] > threshold[i];
  }
}

// ---------- POSITION ----------
float computePosition(bool &lost, bool &edgeDir){
  float sum = 0;
  int count = 0;

  //bool leftEdge = digitalVal[EDGE_L];
  bool rightEdge = digitalVal[EDGE_R];

  for(int i=MID_START;i<=MID_END;i++){
    if(digitalVal[i]){
      sum += (i - 7.5f);
      count++;
    }
  }

  if(count == 0){
    lost = true;
    edgeDir = rightEdge; // deterministic
    return 0;
  }

  lost = false;
  return sum / count;
}

// ---------- PID ----------
float computePID(float error){
  unsigned long now = micros();
  float dt = (now - prevTime) * 1e-6f;
  prevTime = now;

  if(dt <= 0) return 0;

  integral += error * dt;
  integral = constrain(integral, -2.0f, 2.0f); // anti-windup

  float derivative = (error - prevError) / dt;
  dFiltered = 0.7f * dFiltered + 0.3f * derivative;

  prevError = error;
  return KP*error + KI*integral + KD*dFiltered;
}

// ---------- MOTOR ----------
void writeMotors(float pid){
  int L = capPWM(BASE_PWM - pid);
  int R = capPWM(BASE_PWM + pid);

  analogWrite(LM, L); digitalWrite(LMN, HIGH);
  analogWrite(RM, R); digitalWrite(RMN, HIGH);
}
// ---------- SENSOR CALIBRATION & EEPROM SAVE/LOAD ----------
// Add this to your line_follower.ino (above setup or below helpers)



// Calibration settings
constexpr unsigned long CAL_DURATION_MS   = 6000UL; // how long to sample during auto-calibration
constexpr uint8_t         SAMPLES_PER_READ = 5;     // samples per sensor read (median filter)

// EEPROM layout (safe small footprint)
constexpr int EEPROM_MAGIC_ADDR = 0;
constexpr uint32_t EEPROM_MAGIC = 0x4C465231; // "LFR1"
constexpr int EEPROM_BASE = 4; // start address after magic
// we store min/max as uint16_t for each sensor: 2 * TOTAL_SENSORS * 2 bytes
// size = 4 + 4 + (TOTAL_SENSORS * 4)

uint16_t calMin[TOTAL_SENSORS];
uint16_t calMax[TOTAL_SENSORS];
int calThreshold[TOTAL_SENSORS]; // computed (min+max)/2

// simple median for small odd sample counts
int medianOf(int arr[], int n){
  // insertion sort small array
  for(int i=1;i<n;i++){
    int v=arr[i], j=i-1;
    while(j>=0 && arr[j]>v){ arr[j+1]=arr[j]; j--; }
    arr[j+1]=v;
  }
  return arr[n/2];
}

// read a single sensor with minor smoothing -> median of several analog reads
int readRawSmoothed(int idx){
  int s[SAMPLES_PER_READ];
  for(int k=0;k<SAMPLES_PER_READ;k++){
    s[k] = analogRead(sensorPins[idx]);
    // tiny delay can help if your sensors/adc are noisy; keep small for speed:
    // delayMicroseconds(50);
  }
  return medianOf(s, SAMPLES_PER_READ);
}

// Auto-calibration routine (blocking, run in setup or on a button press)
void autoCalibrate(unsigned long duration_ms = CAL_DURATION_MS){
  // initialize
  for(int i=0;i<TOTAL_SENSORS;i++){
    calMin[i] = 0xFFFF;
    calMax[i] = 0;
  }

  unsigned long start = millis();
  while(millis() - start < duration_ms){
    for(int i=0;i<TOTAL_SENSORS;i++){
      int v = readRawSmoothed(i);
      if(v < calMin[i]) calMin[i] = v;
      if(v > calMax[i]) calMax[i] = v;
    }
    // brief yield to allow other things (if using soft-RTOS) - not needed normally:
    // delay(1);
  }

  // If any sensor never changed (min still 0xFFFF), set sensible defaults
  for(int i=0;i<TOTAL_SENSORS;i++){
    if(calMin[i] == 0xFFFF) calMin[i] = 0;          // fallback
    if(calMax[i] <= calMin[i]) calMax[i] = calMin[i] + 20; // small headroom
    // compute threshold halfway (you can bias toward black or white if needed)
    calThreshold[i] = (calMin[i] + calMax[i]) / 2;
  }

  saveCalibrationToEEPROM();
}

// Convert raw analog reading to calibrated 0..1000 (higher = darker)
// Mirrors Pololu's QTR mapping (0..1000) which is convenient for PID scaling.
int readCalibrated(int idx){
  int raw = readRawSmoothed(idx);

  // clamp to min/max to avoid negative mapping
  uint16_t lo = calMin[idx];
  uint16_t hi = calMax[idx];
  if(raw <= lo) return 1000;           // treat as black
  if(raw >= hi) return 0;              // treat as white

  // map raw from [lo..hi] -> [1000..0]
  long scaled = (long)(raw - lo) * 1000L / (long)(hi - lo);
  int out = 1000 - (int)scaled;
  return out;
}

// Fill digitalVal[] using calibrated threshold
void readSensorsCalibrated(){
  for(int i=0;i<TOTAL_SENSORS;i++){
    int cal = readCalibrated(i); // 0..1000, 1000 = black
    // digital decision using threshold on calibrated scale (e.g., 500)
    digitalVal[i] = (cal >= 500);
    analogVal[i]  = cal; // store calibrated numeric value if you want (0..1000)
  }
}

// EEPROM save/load helpers
void saveCalibrationToEEPROM(){
  // magic + min/max pairs
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  int addr = EEPROM_BASE;
  for(int i=0;i<TOTAL_SENSORS;i++){
    EEPROM.put(addr, calMin[i]); addr += sizeof(uint16_t);
    EEPROM.put(addr, calMax[i]); addr += sizeof(uint16_t);
  }
  // optional: store thresholds too (not necessary - derived)
}

bool loadCalibrationFromEEPROM(){
  uint32_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if(magic != EEPROM_MAGIC) return false;
  int addr = EEPROM_BASE;
  for(int i=0;i<TOTAL_SENSORS;i++){
    EEPROM.get(addr, calMin[i]); addr += sizeof(uint16_t);
    EEPROM.get(addr, calMax[i]); addr += sizeof(uint16_t);
    if(calMax[i] <= calMin[i]) { // invalid data -> fail
      return false;
    }
    calThreshold[i] = (calMin[i] + calMax[i]) / 2;
  }
  return true;
}

// ---------- SETUP ----------
void setup(){
  pinMode(LM,OUTPUT); pinMode(LMN,OUTPUT);
  pinMode(RM,OUTPUT); pinMode(RMN,OUTPUT);

    // try load calibration; if missing, auto-calibrate
  if(!loadCalibrationFromEEPROM()){
    // Optional: blink an LED or wait for user to place robot on surfaces
    // run calibration now (robot should be moved over black/white area or spun)
    autoCalibrate(CAL_DURATION_MS);
  }


  prevTime = micros();
}
//---------------------Left_at_juction------------------
bool isJunction(){
  int active = 0;
  for(int i = MID_START; i <= MID_END; i++){
    if(digitalVal[i]) active++;
  }
  // both edges + wide middle detection
  return (digitalVal[EDGE_L] && digitalVal[EDGE_R] && active >= 5);
}


// ---------- LOOP ----------
void loop(){
  readSensorsCalibrated();

  bool lost = false;
  bool edgeDir = false;
  float pos = computePosition(lost, edgeDir);
  // ---------- JUNCTION: FORCE LEFT TURN ----------
  if(isJunction()){
    // Left turn: left motor slow, right motor fast
    analogWrite(LM, 60);
    digitalWrite(LMN, HIGH);
    analogWrite(RM, BASE_PWM + 60);
    digitalWrite(RMN, HIGH);
    return;
  }
  // ---------- LINE LOST ----------
  if(lost){
    // deterministic recovery
    digitalWrite(LM, LOW);
    digitalWrite(LMN, HIGH);
    digitalWrite(RM, HIGH);
    digitalWrite(RMN, LOW);
    return;
  }
  // ---------- NORMAL PID FOLLOW ----------
  float pid = computePID(pos);
  writeMotors(pid);
}
