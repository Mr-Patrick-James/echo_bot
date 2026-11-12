#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

const char* ssid     = ".........";
const char* password = "12345678910";

WebServer server(80);

// ── Motor pins ───────────────────────────────────────────
const int IN1 = 26;  // Rear  left  motor
const int IN2 = 25;
const int IN3 = 33;  // Rear  right motor
const int IN4 = 32;

const int IN5 = 18;  // Front left  motor
const int IN6 = 19;
const int IN7 = 4;   // Front right motor
const int IN8 = 17;

// ── Ultrasonic HC-SR04 ───────────────────────────────────
const int TRIG_PIN = 27;
const int ECHO_PIN = 14;

// ── Buzzer ───────────────────────────────────────────────
const int BUZZER_PIN = 23;  // D23

// ── Trash separator (servos) ─────────────────────────────
const int SORT_SERVO_PIN = 15;  // sorting / position servo
const int CONVEYOR_PIN   = 12;  // conveyor (continuous-rotation servo)

// ── Collector TT motor ────────────────────────────────────
const int COLLECTOR_A = 21;  // L298N/L9110S IN_A
const int COLLECTOR_B = 22;  // L298N/L9110S IN_B

const int POS_BIO     =  789;
const int POS_HOME    = 1680;
const int POS_NONBIO  = 2500;
const int SERVO_MIN   =  500;
const int SERVO_MAX   = 2500;

const int MOTOR_UP    = 1000;
const int MOTOR_STOP  = 1500;
const int MOTOR_DOWN  = 2000;

const int RECYCLE_UP_MS    = 700;
const int RECYCLE_DOWN_MS  = 300;
const int RECYCLE_PAUSE_MS = 500;
const int SORT_HOLD_MS     = 1500;

Servo sortServo;
Servo convMotor;
int  trashServoUs = POS_HOME;
int  trashMotorUs = MOTOR_STOP;
bool autoSorting  = false;
unsigned long sortStartMs = 0;
int  recyclState  = 0;   // 0=idle 1=UP 2=pause 3=DOWN

// ── Trash counters ────────────────────────────────────────
int countBio     = 0;
int countNonBio  = 0;
int countRecycle = 0;

// ── CAM tracking state ────────────────────────────────────
bool   trackingTrash  = false;
String trackedLabel   = "";
bool   camScanEnabled = false;  // OFF by default — toggle from UI
bool   camMoveEnabled = false;  // bot movement toward trash
const int TRACK_DRIVE_SPEED = 160;
const int TRACK_TURN_SPEED  = 130;
const int TRACK_STEER_DEAD  = 20;
unsigned long recyclStartMs = 0;

// ── Auto-cycle state machine ──────────────────────────────
enum CycleState {
  CYCLE_IDLE,        // waiting for detection
  CYCLE_LOWERING,    // lifter going to 0, waiting before collector+move start
  CYCLE_COLLECTING,  // driving + collector running toward trash
  CYCLE_LIFT_UP,     // told lifter to go UP, polling /status
  CYCLE_SETTLE,      // lifter reached 1580, waiting 5s to settle
  CYCLE_SEGREGATE,   // triggering servo sort
  CYCLE_DONE         // resuming CAM
};
CycleState    cycleState     = CYCLE_IDLE;
unsigned long cycleTimer     = 0;
String        lastTrashLabel = "";    // last confirmed label for segregation
bool          camPaused      = false; // pauses CAM inference during cycle
const unsigned long SETTLE_MS      = 2000;   // 2s settle after lifter reaches UP
const unsigned long LIFTER_POLL_MS = 600;    // poll lifter status every 600ms
unsigned long lastLifterPoll       = 0;
bool          lifterReachedUp      = false;

// ── PWM config ───────────────────────────────────────────
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;

// ── Mode & Gears ─────────────────────────────────────────
enum Mode { MANUAL, AVOID, FOLLOW };
Mode currentMode = MANUAL;
int currentGear = 4; // Default to Gear 4 (fastest)

// ── Avoid thresholds ─────────────────────────────────────
const int AVOID_STOP_CM    = 20;
const int AVOID_REVERSE_CM = 12;

// ── Follow tuning ────────────────────────────────────────
const int FOLLOW_TARGET_CM = 22;
const int FOLLOW_DEADBAND  = 5;
const int FOLLOW_TOO_FAR   = 60;
const int FOLLOW_MIN_SPEED = 80;
const int FOLLOW_MAX_SPEED = 220;
const int FOLLOW_RAMP_CM   = 25;

// ── Auto timing ──────────────────────────────────────────
unsigned long lastAutoRun = 0;
const int     AUTO_INTERVAL = 120;

enum AvoidState { AV_FORWARD, AV_STOP, AV_REVERSE, AV_TURN };
AvoidState    avoidState = AV_FORWARD;
unsigned long avoidTimer = 0;

// ═══════════════════════════════════════════════════════════
// NOTE DEFINITIONS (full set)
// ═══════════════════════════════════════════════════════════
#define REST     0
#define NOTE_B0   31
#define NOTE_C1   33
#define NOTE_CS1  35
#define NOTE_D1   37
#define NOTE_DS1  39
#define NOTE_E1   41
#define NOTE_F1   44
#define NOTE_FS1  46
#define NOTE_G1   49
#define NOTE_GS1  52
#define NOTE_A1   55
#define NOTE_AS1  58
#define NOTE_B1   62
#define NOTE_C2   65
#define NOTE_CS2  69
#define NOTE_D2   73
#define NOTE_DS2  78
#define NOTE_E2   82
#define NOTE_F2   87
#define NOTE_FS2  93
#define NOTE_G2   98
#define NOTE_GS2  104
#define NOTE_A2   110
#define NOTE_AS2  117
#define NOTE_B2   123
#define NOTE_C3   131
#define NOTE_CS3  139
#define NOTE_D3   147
#define NOTE_DS3  156
#define NOTE_E3   165
#define NOTE_F3   175
#define NOTE_FS3  185
#define NOTE_G3   196
#define NOTE_GS3  208
#define NOTE_A3   220
#define NOTE_AS3  233
#define NOTE_B3   247
#define NOTE_C4   262
#define NOTE_CS4  277
#define NOTE_D4   294
#define NOTE_DS4  311
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_FS4  370
#define NOTE_G4   392
#define NOTE_GS4  415
#define NOTE_A4   440
#define NOTE_AS4  466
#define NOTE_B4   494
#define NOTE_C5   523
#define NOTE_CS5  554
#define NOTE_D5   587
#define NOTE_DS5  622
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_FS5  740
#define NOTE_G5   784
#define NOTE_GS5  831
#define NOTE_A5   880
#define NOTE_AS5  932
#define NOTE_B5   988
#define NOTE_C6   1047
#define NOTE_CS6  1109
#define NOTE_D6   1175
#define NOTE_DS6  1245
#define NOTE_E6   1319
#define NOTE_F6   1397
#define NOTE_FS6  1480
#define NOTE_G6   1568
#define NOTE_GS6  1661
#define NOTE_A6   1760
#define NOTE_AS6  1865
#define NOTE_B6   1976
#define NOTE_C7   2093
#define NOTE_CS7  2217
#define NOTE_D7   2349
#define NOTE_DS7  2489
#define NOTE_E7   2637
#define NOTE_F7   2794
#define NOTE_FS7  2960
#define NOTE_G7   3136
#define NOTE_GS7  3322
#define NOTE_A7   3520
#define NOTE_AS7  3729
#define NOTE_B7   3951
#define NOTE_C8   4186

// ── Melody: Super Mario Bros (full) ──────────────────────
int melodyMario[] = {
  NOTE_E5,8, NOTE_E5,8, REST,8, NOTE_E5,8, REST,8, NOTE_C5,8, NOTE_E5,8,
  NOTE_G5,4, REST,4, NOTE_G4,8, REST,4,
  NOTE_C5,-4, NOTE_G4,8, REST,4, NOTE_E4,-4,
  NOTE_A4,4, NOTE_B4,4, NOTE_AS4,8, NOTE_A4,4,
  NOTE_G4,-8, NOTE_E5,-8, NOTE_G5,-8, NOTE_A5,4, NOTE_F5,8, NOTE_G5,8,
  REST,8, NOTE_E5,4, NOTE_C5,8, NOTE_D5,8, NOTE_B4,-4,
  NOTE_C5,-4, NOTE_G4,8, REST,4, NOTE_E4,-4,
  NOTE_A4,4, NOTE_B4,4, NOTE_AS4,8, NOTE_A4,4,
  NOTE_G4,-8, NOTE_E5,-8, NOTE_G5,-8, NOTE_A5,4, NOTE_F5,8, NOTE_G5,8,
  REST,8, NOTE_E5,4, NOTE_C5,8, NOTE_D5,8, NOTE_B4,-4,
  REST,4, NOTE_G5,8, NOTE_FS5,8, NOTE_F5,8, NOTE_DS5,4, NOTE_E5,8,
  REST,8, NOTE_GS4,8, NOTE_A4,8, NOTE_C4,8, REST,8, NOTE_A4,8, NOTE_C5,8, NOTE_D5,8,
  REST,4, NOTE_DS5,4, REST,8, NOTE_D5,-4,
  NOTE_C5,2, REST,2,
  REST,4, NOTE_G5,8, NOTE_FS5,8, NOTE_F5,8, NOTE_DS5,4, NOTE_E5,8,
  REST,8, NOTE_GS4,8, NOTE_A4,8, NOTE_C4,8, REST,8, NOTE_A4,8, NOTE_C5,8, NOTE_D5,8,
  REST,4, NOTE_DS5,4, REST,8, NOTE_D5,-4,
  NOTE_C5,2, REST,2,
  NOTE_C5,8, NOTE_C5,4, NOTE_C5,8, REST,8, NOTE_C5,8, NOTE_D5,4,
  NOTE_E5,8, NOTE_C5,4, NOTE_A4,8, NOTE_G4,2,
  NOTE_C5,8, NOTE_C5,4, NOTE_C5,8, REST,8, NOTE_C5,8, NOTE_D5,8, NOTE_E5,8,
  REST,1,
  NOTE_C5,8, NOTE_C5,4, NOTE_C5,8, REST,8, NOTE_C5,8, NOTE_D5,4,
  NOTE_E5,8, NOTE_C5,4, NOTE_A4,8, NOTE_G4,2,
  NOTE_E5,8, NOTE_E5,8, REST,8, NOTE_E5,8, REST,8, NOTE_C5,8, NOTE_E5,4,
  NOTE_G5,4, REST,4, NOTE_G4,4, REST,4,
  NOTE_C5,-4, NOTE_G4,8, REST,4, NOTE_E4,-4,
  NOTE_A4,4, NOTE_B4,4, NOTE_AS4,8, NOTE_A4,4,
  NOTE_G4,-8, NOTE_E5,-8, NOTE_G5,-8, NOTE_A5,4, NOTE_F5,8, NOTE_G5,8,
  REST,8, NOTE_E5,4, NOTE_C5,8, NOTE_D5,8, NOTE_B4,-4,
  NOTE_C5,-4, NOTE_G4,8, REST,4, NOTE_E4,-4,
  NOTE_A4,4, NOTE_B4,4, NOTE_AS4,8, NOTE_A4,4,
  NOTE_G4,-8, NOTE_E5,-8, NOTE_G5,-8, NOTE_A5,4, NOTE_F5,8, NOTE_G5,8,
  REST,8, NOTE_E5,4, NOTE_C5,8, NOTE_D5,8, NOTE_B4,-4,
  NOTE_C5,-4, NOTE_G4,-4, NOTE_E4,4,
  NOTE_A4,-8, NOTE_B4,-8, NOTE_A4,-8, NOTE_GS4,-8, NOTE_AS4,-8, NOTE_GS4,-8,
  NOTE_G4,8, NOTE_D4,8, NOTE_E4,-2,
};
const int tempoMario = 200;

// ── Melody: Never Gonna Give You Up (full) ───────────────
int melodyRick[] = {
  NOTE_D5,8, NOTE_E5,8, NOTE_A4,4, NOTE_E5,8, NOTE_FS5,8, NOTE_A5,4,
  NOTE_D5,8, NOTE_E5,8, NOTE_A4,4, REST,4,
  NOTE_D5,8, NOTE_E5,8, NOTE_A4,2,
  NOTE_A4,8, NOTE_B4,8, NOTE_D5,8, NOTE_B4,8,
  NOTE_FS5,-4, NOTE_FS5,4,
  NOTE_E5,-2,
  NOTE_A4,8, NOTE_B4,8, NOTE_D5,8, NOTE_B4,8,
  NOTE_E5,-4, NOTE_E5,4,
  NOTE_CS5,-8, NOTE_B4,-8,
  NOTE_A4,-2,
  NOTE_A4,8, NOTE_B4,8, NOTE_D5,8, NOTE_B4,8,
  NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-4,
  NOTE_B4,8, REST,8,
  NOTE_A4,4, NOTE_A4,8, NOTE_B4,8, NOTE_D5,8, NOTE_CS5,8,
  NOTE_B4,2,
  NOTE_A4,8, NOTE_B4,8, NOTE_D5,8, NOTE_B4,8,
  NOTE_FS5,-4, NOTE_FS5,4,
  NOTE_E5,-2,
  NOTE_A4,8, NOTE_B4,8, NOTE_D5,8, NOTE_B4,8,
  NOTE_E5,-4, NOTE_E5,4,
  NOTE_CS5,-8, NOTE_B4,-8,
  NOTE_A4,-2,
  NOTE_A4,8, NOTE_B4,8, NOTE_D5,8, NOTE_B4,8,
  NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-4,
  NOTE_B4,8, REST,8,
  NOTE_A4,4, REST,2,
};
const int tempoRick = 114;

// ── Melody: Imperial March (full) ────────────────────────
int melodyImperial[] = {
  NOTE_A4,4, NOTE_A4,4, NOTE_A4,4,
  NOTE_F4,-8, NOTE_C5,16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
  NOTE_E5,4, NOTE_E5,4, NOTE_E5,4,
  NOTE_F5,-8, NOTE_C5,16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
  NOTE_A5,4, NOTE_A4,-8, NOTE_A4,16, NOTE_A5,4,
  NOTE_GS5,-8, NOTE_G5,16,
  NOTE_FS5,16, NOTE_F5,16, NOTE_FS5,8, REST,8,
  NOTE_AS4,8, NOTE_DS5,4, NOTE_D5,-8, NOTE_CS5,16,
  NOTE_C5,16, NOTE_B4,16, NOTE_C5,8, REST,8,
  NOTE_F4,8, NOTE_GS4,4, NOTE_F4,-8, NOTE_A4,16,
  NOTE_C5,4, NOTE_A4,-8, NOTE_C5,16, NOTE_E5,2,
  NOTE_A5,4, NOTE_A4,-8, NOTE_A4,16, NOTE_A5,4,
  NOTE_GS5,-8, NOTE_G5,16,
  NOTE_FS5,16, NOTE_F5,16, NOTE_FS5,8, REST,8,
  NOTE_AS4,8, NOTE_DS5,4, NOTE_D5,-8, NOTE_CS5,16,
  NOTE_C5,16, NOTE_B4,16, NOTE_C5,8, REST,8,
  NOTE_F4,8, NOTE_GS4,4, NOTE_F4,-8, NOTE_AS4,16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
};
const int tempoImperial = 120;

// ── Melody: Tetris (full) ────────────────────────────────
int melodyTetris[] = {
  NOTE_E5,4, NOTE_B4,8, NOTE_C5,8, NOTE_D5,4, NOTE_C5,8, NOTE_B4,8,
  NOTE_A4,4, NOTE_A4,8, NOTE_C5,8, NOTE_E5,4, NOTE_D5,8, NOTE_C5,8,
  NOTE_B4,-4, NOTE_C5,8, NOTE_D5,4, NOTE_E5,4,
  NOTE_C5,4, NOTE_A4,4, NOTE_A4,4, REST,4,
  REST,8, NOTE_D5,4, NOTE_F5,8, NOTE_A5,4, NOTE_G5,8, NOTE_F5,8,
  NOTE_E5,-4, NOTE_C5,8, NOTE_E5,4, NOTE_D5,8, NOTE_C5,8,
  NOTE_B4,4, NOTE_B4,8, NOTE_C5,8, NOTE_D5,4, NOTE_E5,4,
  NOTE_C5,4, NOTE_A4,4, NOTE_A4,4, REST,4,
  NOTE_E5,4, NOTE_B4,8, NOTE_C5,8, NOTE_D5,4, NOTE_C5,8, NOTE_B4,8,
  NOTE_A4,4, NOTE_A4,8, NOTE_C5,8, NOTE_E5,4, NOTE_D5,8, NOTE_C5,8,
  NOTE_B4,-4, NOTE_C5,8, NOTE_D5,4, NOTE_E5,4,
  NOTE_C5,4, NOTE_A4,4, NOTE_A4,4, REST,4,
};
const int tempoTetris = 144;

// ── Melody: Happy Birthday (full) ────────────────────────
int melodyHappy[] = {
  NOTE_C4,8, NOTE_C4,8, NOTE_D4,4, NOTE_C4,4, NOTE_F4,4, NOTE_E4,2,
  NOTE_C4,8, NOTE_C4,8, NOTE_D4,4, NOTE_C4,4, NOTE_G4,4, NOTE_F4,2,
  NOTE_C4,8, NOTE_C4,8, NOTE_C5,4, NOTE_A4,4, NOTE_F4,4, NOTE_E4,4, NOTE_D4,4,
  NOTE_AS4,8, NOTE_AS4,8, NOTE_A4,4, NOTE_F4,4, NOTE_G4,4, NOTE_F4,2,
};
const int tempoHappy = 140;

// ── Melody: Jingle Bells (full) ──────────────────────────
int melodyJingle[] = {
  NOTE_E4,4, NOTE_E4,4, NOTE_E4,2,
  NOTE_E4,4, NOTE_E4,4, NOTE_E4,2,
  NOTE_E4,4, NOTE_G4,4, NOTE_C4,-4, NOTE_D4,8, NOTE_E4,1,
  NOTE_F4,4, NOTE_F4,4, NOTE_F4,-4, NOTE_F4,8,
  NOTE_F4,4, NOTE_E4,4, NOTE_E4,4, NOTE_E4,8, NOTE_E4,8,
  NOTE_E4,4, NOTE_D4,4, NOTE_D4,4, NOTE_E4,4, NOTE_D4,2, NOTE_G4,2,
  NOTE_E4,4, NOTE_E4,4, NOTE_E4,2,
  NOTE_E4,4, NOTE_E4,4, NOTE_E4,2,
  NOTE_E4,4, NOTE_G4,4, NOTE_C4,-4, NOTE_D4,8, NOTE_E4,1,
  NOTE_F4,4, NOTE_F4,4, NOTE_F4,-4, NOTE_F4,8,
  NOTE_F4,4, NOTE_E4,4, NOTE_E4,4, NOTE_E4,8, NOTE_E4,8,
  NOTE_G4,4, NOTE_G4,4, NOTE_F4,4, NOTE_D4,4, NOTE_C4,1,
};
const int tempoJingle = 160;

// ═══════════════════════════════════════════════════════════
// NON-BLOCKING MELODY PLAYER
// ═══════════════════════════════════════════════════════════
int*  activeMelody    = nullptr;
int   activeMelodyLen = 0;
int   activeTempo     = 120;
int   noteIndex       = 0;
unsigned long noteStart    = 0;
unsigned long noteDuration = 0;
bool  songPlaying     = false;

void startSong(int* melody, int melodySize, int tempo) {
  noTone(BUZZER_PIN);
  activeMelody    = melody;
  activeMelodyLen = melodySize / 2;
  activeTempo     = tempo;
  noteIndex       = 0;
  songPlaying     = true;
  noteStart       = millis();
  noteDuration    = 0;
}

void stopSong() {
  songPlaying = false;
  noTone(BUZZER_PIN);
}

void updateSong() {
  if (!songPlaying) return;
  unsigned long now = millis();
  if (now - noteStart < noteDuration) return;

  noTone(BUZZER_PIN);
  if (noteIndex >= activeMelodyLen) { songPlaying = false; return; }

  int freq    = activeMelody[noteIndex * 2];
  int divider = activeMelody[noteIndex * 2 + 1];
  int whole   = (60000 * 4) / activeTempo;

  if (divider > 0) noteDuration = whole / divider;
  else             noteDuration = (whole / abs(divider)) * 3 / 2;

  if (freq != REST) tone(BUZZER_PIN, freq, noteDuration * 9 / 10);

  noteStart = now;
  noteIndex++;
}

// ── Debug log ────────────────────────────────────────────
#define LOG_SIZE 10
String debugLog[LOG_SIZE];
int logCount = 0;
void addLog(const String& msg) {
  for (int i = LOG_SIZE - 1; i > 0; i--) debugLog[i] = debugLog[i-1];
  debugLog[0] = msg;
  if (logCount < LOG_SIZE) logCount++;
  Serial.println("[LOG] " + msg);
}

// ═══════════════════════════════════════════════════════════
// TRASH SEPARATOR HELPERS
// ═══════════════════════════════════════════════════════════
void trashSetServo(int us) {
  trashServoUs = constrain(us, SERVO_MIN, SERVO_MAX);
  sortServo.writeMicroseconds(trashServoUs);
}
void trashSetMotor(int us) {
  trashMotorUs = constrain(us, 500, 2500);
  convMotor.writeMicroseconds(trashMotorUs);
}
String trashServoLabel(int us) {
  if (us <= POS_BIO     + 60) return "BIO";
  if (us >= POS_NONBIO  - 60) return "NON-BIO";
  if (abs(us - POS_HOME) <= 100) return "HOME";
  return String(us) + "us";
}
String trashMotorLabel(int us) {
  if (abs(us - MOTOR_UP)   <= 60) return "UP";
  if (abs(us - MOTOR_DOWN) <= 60) return "DOWN";
  if (abs(us - MOTOR_STOP) <= 60) return "STOP";
  return String(us) + "us";
}
String trashStatusJson() {
  String j = "{";
  j += "\"servo\":"      + String(trashServoUs);
  j += ",\"sname\":\""   + trashServoLabel(trashServoUs) + "\"";
  j += ",\"motor\":"     + String(trashMotorUs);
  j += ",\"mname\":\""   + trashMotorLabel(trashMotorUs) + "\"";
  j += ",\"sorting\":"   + String(autoSorting     ? "true" : "false");
  j += ",\"recycling\":" + String(recyclState > 0 ? "true" : "false");
  j += ",\"rstate\":"    + String(recyclState);
  j += ",\"atHome\":"    + String(abs(trashServoUs - POS_HOME) <= 100 ? "true" : "false");
  j += ",\"log\":[";
  for (int i = 0; i < logCount; i++) {
    if (i) j += ",";
    String s = debugLog[i]; s.replace("\"","'");
    j += "\"" + s + "\"";
  }
  j += "]}";
  return j;
}

// ═══════════════════════════════════════════════════════════
// MOTOR HELPERS
// ═══════════════════════════════════════════════════════════
void setupPWM() {
  ledcAttach(IN2, PWM_FREQ, PWM_RES);  // rear  left  forward
  ledcAttach(IN4, PWM_FREQ, PWM_RES);  // rear  right forward
  ledcAttach(IN6, PWM_FREQ, PWM_RES);  // front left  forward
  ledcAttach(IN8, PWM_FREQ, PWM_RES);  // front right forward
  pinMode(IN1, OUTPUT); pinMode(IN3, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN7, OUTPUT);
  digitalWrite(IN1, LOW); ledcWrite(IN2, 0);
  digitalWrite(IN3, LOW); ledcWrite(IN4, 0);
  digitalWrite(IN5, LOW); ledcWrite(IN6, 0);
  digitalWrite(IN7, LOW); ledcWrite(IN8, 0);

  // ── Collector TT motor setup ──────────────────────────
  ledcAttach(COLLECTOR_A, PWM_FREQ, PWM_RES);
  ledcAttach(COLLECTOR_B, PWM_FREQ, PWM_RES);
  ledcWrite(COLLECTOR_A, 0);
  ledcWrite(COLLECTOR_B, 0);
}

// ── Collector TT motor control ────────────────────────────
// speed: 0-255, direction: FORWARD / BACKWARD / STOP
void collectorForward(int speed = 200) {
  ledcWrite(COLLECTOR_A, 0);
  ledcWrite(COLLECTOR_B, speed);
}
void collectorBackward(int speed = 200) {
  ledcWrite(COLLECTOR_A, speed);
  ledcWrite(COLLECTOR_B, 0);
}
void collectorStop() {
  ledcWrite(COLLECTOR_A, 0);
  ledcWrite(COLLECTOR_B, 0);
}

void setLeft(int speed) {       // rear left
  if (speed >= 0) { digitalWrite(IN1, LOW);  ledcWrite(IN2, speed); }
  else            { digitalWrite(IN1, HIGH); ledcWrite(IN2, 255 + speed); }
}

void setRight(int speed) {      // rear right
  if (speed >= 0) { digitalWrite(IN3, LOW);  ledcWrite(IN4, speed); }
  else            { digitalWrite(IN3, HIGH); ledcWrite(IN4, 255 + speed); }
}

void setFrontLeft(int speed) {  // front left
  if (speed >= 0) { digitalWrite(IN5, LOW);  ledcWrite(IN6, speed); }
  else            { digitalWrite(IN5, HIGH); ledcWrite(IN6, 255 + speed); }
}

void setFrontRight(int speed) { // front right
  if (speed >= 0) { digitalWrite(IN7, LOW);  ledcWrite(IN8, speed); }
  else            { digitalWrite(IN7, HIGH); ledcWrite(IN8, 255 + speed); }
}

void motorStop() {
  setLeft(0); setRight(0);
  setFrontLeft(0); setFrontRight(0);
}
void motorForward(int s = 200) {
  setLeft(s);   setRight(s);
  setFrontLeft(-s); setFrontRight(-s);
}
void motorReverse(int s = 180) {
  setLeft(-s);  setRight(-s);
  setFrontLeft(s); setFrontRight(s);
}
void motorSpinLeft(int s = 180) {
  setLeft(-s);  setRight(s);
  setFrontLeft(s); setFrontRight(-s);
}
void motorSpinRight(int s = 180) {
  setLeft(s);   setRight(-s);
  setFrontLeft(-s); setFrontRight(s);
}

// ═══════════════════════════════════════════════════════════
// ULTRASONIC
// ═══════════════════════════════════════════════════════════
long readRaw() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 25000);
  return dur == 0 ? 400 : dur / 58;
}

long readDistanceCM() {
  long a = readRaw(); delayMicroseconds(500);
  long b = readRaw(); delayMicroseconds(500);
  long c = readRaw();
  if (a > b) { long t=a; a=b; b=t; }
  if (b > c) { long t=b; b=c; c=t; }
  if (a > b) { long t=a; a=b; b=t; }
  return b;
}

// ═══════════════════════════════════════════════════════════
// AUTONOMOUS LOGIC
// ═══════════════════════════════════════════════════════════
void runAvoid() {
  long dist = readDistanceCM();
  switch (avoidState) {
    case AV_FORWARD:
      if (dist < AVOID_REVERSE_CM) {
        motorReverse(); avoidState = AV_REVERSE; avoidTimer = millis();
      } else if (dist < AVOID_STOP_CM) {
        motorStop(); avoidState = AV_TURN; avoidTimer = millis();
      } else {
        motorForward();
      }
      break;
    case AV_REVERSE:
      if (millis() - avoidTimer > 500) { motorStop(); avoidState = AV_TURN; avoidTimer = millis(); }
      break;
    case AV_TURN:
      motorSpinRight();
      if (millis() - avoidTimer > 600) avoidState = AV_FORWARD;
      break;
    default: avoidState = AV_FORWARD; break;
  }
}

void runFollow() {
  long dist  = readDistanceCM();
  if (dist > FOLLOW_TOO_FAR) { motorStop(); return; }
  long error = dist - FOLLOW_TARGET_CM;
  if (abs(error) <= FOLLOW_DEADBAND) { motorStop(); return; }
  int spd = map(constrain(abs(error), FOLLOW_DEADBAND, FOLLOW_RAMP_CM),
                FOLLOW_DEADBAND, FOLLOW_RAMP_CM, FOLLOW_MIN_SPEED, FOLLOW_MAX_SPEED);
  if (error > 0) motorForward(spd);
  else           motorReverse(spd * 7 / 10);
}

// ── Lifter HTTP — fully non-blocking background task ─────
// All HTTP to ESP8266 goes through here — main loop never blocks

volatile bool lifterMoving     = true;
volatile bool lifterPollActive = false;
SemaphoreHandle_t lifterMutex  = NULL;

// Command queue — main loop queues commands, task executes them
#define LIFTER_CMD_NONE    0
#define LIFTER_CMD_GOHOME  1
#define LIFTER_CMD_GOUP    2
volatile int lifterPendingCmd  = LIFTER_CMD_NONE;

void lifterPollTask(void* arg) {
  for (;;) {
    // Execute any pending command first
    int cmd = LIFTER_CMD_NONE;
    if (lifterMutex) xSemaphoreTake(lifterMutex, portMAX_DELAY);
    cmd = lifterPendingCmd;
    lifterPendingCmd = LIFTER_CMD_NONE;
    if (lifterMutex) xSemaphoreGive(lifterMutex);

    if (cmd == LIFTER_CMD_GOHOME) {
      HTTPClient hc;
      hc.begin("http://10.60.114.130/goHome");
      hc.setTimeout(400); hc.GET(); hc.end();
    } else if (cmd == LIFTER_CMD_GOUP) {
      HTTPClient hc;
      hc.begin("http://10.60.114.130/goUp");
      hc.setTimeout(400); hc.GET(); hc.end();
    }

    // Poll status if needed
    if (lifterPollActive) {
      HTTPClient hc;
      hc.begin("http://10.60.114.130/status");
      hc.setTimeout(400);
      int code = hc.GET();
      bool moving = true;
      if (code == 200) {
        String body = hc.getString();
        moving = (body.indexOf("\"moving\":false") < 0);
      }
      hc.end();
      if (lifterMutex) {
        xSemaphoreTake(lifterMutex, portMAX_DELAY);
        lifterMoving = moving;
        xSemaphoreGive(lifterMutex);
      }
    }

    vTaskDelay(600 / portTICK_PERIOD_MS);
  }
}

// Safe helpers — called from main loop/routes, never block
void lifterGoHome() {
  if (lifterMutex) xSemaphoreTake(lifterMutex, portMAX_DELAY);
  lifterPendingCmd = LIFTER_CMD_GOHOME;
  lifterMoving     = true;
  lifterPollActive = true;
  if (lifterMutex) xSemaphoreGive(lifterMutex);
}
void lifterGoUp() {
  if (lifterMutex) xSemaphoreTake(lifterMutex, portMAX_DELAY);
  lifterPendingCmd = LIFTER_CMD_GOUP;
  lifterMoving     = true;
  lifterPollActive = true;
  if (lifterMutex) xSemaphoreGive(lifterMutex);
}
bool lifterIsMoving() {
  bool m = true;
  if (lifterMutex) {
    xSemaphoreTake(lifterMutex, portMAX_DELAY);
    m = lifterMoving;
    xSemaphoreGive(lifterMutex);
  }
  return m;
}

// ═══════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN5, OUTPUT); digitalWrite(IN5, LOW);
  pinMode(IN7, OUTPUT); digitalWrite(IN7, LOW);

  // ── Trash separator servos FIRST — must attach before setupPWM() ──
  // ESP32Servo and ledcAttach() share the same ledc channel pool.
  // Attaching servos first reserves their channels; drive motors take the rest.
  // Use ledc channels 4 & 5 explicitly to avoid conflict with drive motor channels 0-3
  sortServo.setPeriodHertz(50);
  convMotor.setPeriodHertz(50);
  sortServo.attach(SORT_SERVO_PIN, SERVO_MIN, SERVO_MAX);
  convMotor.attach(CONVEYOR_PIN,   500,       2500);
  trashSetServo(POS_HOME);
  trashSetMotor(MOTOR_STOP);
  addLog("BOOT OK - servo=HOME motor=STOP");

  setupPWM();
  motorStop();

  pinMode(TRIG_PIN,   OUTPUT);
  pinMode(ECHO_PIN,   INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected. IP:"); Serial.println(WiFi.localIP());
  addLog("WiFi OK IP=" + WiFi.localIP().toString());

  // ── Mode routes ──
  server.on("/MODE_MANUAL", []() {
    currentMode = MANUAL; avoidState = AV_FORWARD; motorStop();
    server.send(200, "text/plain", "MANUAL");
  });
  server.on("/MODE_AVOID",  []() {
    currentMode = AVOID; avoidState = AV_FORWARD;
    server.send(200, "text/plain", "AVOID");
  });
  server.on("/MODE_FOLLOW", []() {
    currentMode = FOLLOW;
    server.send(200, "text/plain", "FOLLOW");
  });

  // ── Gear selection route ──
  server.on("/gear", []() {
    if (server.hasArg("g")) {
      currentGear = server.arg("g").toInt();
      currentGear = constrain(currentGear, 1, 4);
    }
    server.send(200, "text/plain", "OK");
  });

  // ── Joystick ──
  server.on("/joy", []() {
    if (currentMode != MANUAL) { server.send(200, "text/plain", "IGNORED"); return; }
    if (server.hasArg("x") && server.hasArg("y")) {
      int jx = server.arg("x").toInt();  // -100 to 100
      int jy = server.arg("y").toInt();  // -100 to 100

      // Gear multiplier configuration (1 = slowest, 4 = fastest)
      float gearMultiplier = 0.40; // Gear 1 (40%)
      if (currentGear == 2) gearMultiplier = 0.65; // Gear 2 (65%)
      if (currentGear == 3) gearMultiplier = 0.85; // Gear 3 (85%)
      if (currentGear == 4) gearMultiplier = 1.00; // Gear 4 (100%)

      // Scale base PWM range then apply gear multiplier
      int lPWM  = map(constrain(jy - jx, -100, 100), -100, 100, -255, 255) * gearMultiplier;
      int rPWM  = map(constrain(jy + jx, -100, 100), -100, 100, -255, 255) * gearMultiplier;
      int fLPWM = map(constrain(jy + jx, -100, 100), -100, 100, -255, 255) * gearMultiplier;
      int fRPWM = map(constrain(jy - jx, -100, 100), -100, 100, -255, 255) * gearMultiplier;

      setLeft(lPWM);        setRight(rPWM);
      setFrontLeft(-fLPWM);  setFrontRight(-fRPWM);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/STOP", []() {
    if (currentMode == MANUAL) motorStop();
    server.send(200, "text/plain", "OK");
  });

  server.on("/dist", []() {
    server.send(200, "text/plain", String(readDistanceCM()));
  });

  // ── Song routes ──
  server.on("/SONG_MARIO",    []() { startSong(melodyMario,     sizeof(melodyMario)/sizeof(int),    tempoMario);    server.send(200,"text/plain","OK"); });
  server.on("/SONG_RICK",     []() { startSong(melodyRick,      sizeof(melodyRick)/sizeof(int),     tempoRick);     server.send(200,"text/plain","OK"); });
  server.on("/SONG_IMPERIAL", []() { startSong(melodyImperial, sizeof(melodyImperial)/sizeof(int), tempoImperial); server.send(200,"text/plain","OK"); });
  server.on("/SONG_TETRIS",   []() { startSong(melodyTetris,   sizeof(melodyTetris)/sizeof(int),   tempoTetris);   server.send(200,"text/plain","OK"); });
  server.on("/SONG_JINGLE",   []() { startSong(melodyJingle,   sizeof(melodyJingle)/sizeof(int),   tempoJingle);   server.send(200,"text/plain","OK"); });
  server.on("/SONG_HAPPY",    []() { startSong(melodyHappy,     sizeof(melodyHappy)/sizeof(int),    tempoHappy);    server.send(200,"text/plain","OK"); });
  server.on("/SONG_STOP",     []() { stopSong(); server.send(200,"text/plain","OK"); });

  // ── Trash separator routes ──
  server.on("/doBio", []() {
    trashSetServo(POS_BIO);
    addLog("BIO -> servo LEFT " + String(POS_BIO) + "us");
    server.send(200, "application/json", trashStatusJson());
  });
  server.on("/doNonBio", []() {
    trashSetServo(POS_NONBIO);
    addLog("NON-BIO -> servo RIGHT " + String(POS_NONBIO) + "us");
    server.send(200, "application/json", trashStatusJson());
  });
  server.on("/doHome", []() {
    trashSetServo(POS_HOME);
    trashSetMotor(MOTOR_STOP);
    addLog("HOME -> servo CENTER + motor STOP");
    server.send(200, "application/json", trashStatusJson());
  });
  server.on("/doRecyclable", []() {
    if (abs(trashServoUs - POS_HOME) > 100) {
      addLog("RECYCLABLE BLOCKED - servo not at HOME (" + String(trashServoUs) + "us)");
      String j = trashStatusJson();
      j = j.substring(0, j.length()-1) + ",\"blocked\":true}";
      server.send(200, "application/json", j);
      return;
    }
    if (recyclState > 0) { server.send(200, "application/json", trashStatusJson()); return; }
    trashSetMotor(MOTOR_UP);
    recyclState   = 1;
    recyclStartMs = millis();
    addLog("RECYCLABLE -> motor UP, cycle started");
    server.send(200, "application/json", trashStatusJson());
  });
  server.on("/sort", []() {
    trashSetServo(POS_BIO);
    autoSorting = true;
    sortStartMs = millis();
    addLog("AUTO-SORT -> BIO, holding " + String(SORT_HOLD_MS) + "ms");
    server.send(200, "application/json", trashStatusJson());
  });
  server.on("/trashStatus", []() {
    server.send(200, "application/json", trashStatusJson());
  });

  // ── Collector TT motor routes ─────────────────────────────
  server.on("/collector", []() {
    if (server.hasArg("dir")) {
      String dir = server.arg("dir");
      int spd = server.hasArg("spd") ? server.arg("spd").toInt() : 200;
      spd = constrain(spd, 0, 255);
      if      (dir == "fwd")  collectorForward(spd);
      else if (dir == "bwd")  collectorBackward(spd);
      else                    collectorStop();
    }
    server.send(200, "text/plain", "OK");
  });

  // ── CAM tracking routes ───────────────────────────────────
  // Called by ESP32-CAM every inference cycle when trash is detected.
  // ?label=bio|nonbio|recycle  &steer=-100..100  &close=0|1
  server.on("/trackTrash", []() {
    // Ignore if disabled OR cycle already running (not IDLE or LOWERING)
    if (!camScanEnabled && !camMoveEnabled) {
      server.send(200, "text/plain", "DISABLED"); return;
    }
    if (!server.hasArg("steer") || !server.hasArg("close") || !server.hasArg("label")) {
      server.send(400, "text/plain", "Missing args"); return;
    }

    int   steer   = server.arg("steer").toInt();
    trackedLabel   = server.arg("label");
    lastTrashLabel = trackedLabel;
    trackingTrash  = true;
    currentMode    = MANUAL;

    // First detection — tell lifter to go HOME (0) first
    if (cycleState == CYCLE_IDLE) {
      cycleState = CYCLE_LOWERING;
      lifterGoHome();  // non-blocking
      addLog("CYCLE START: " + lastTrashLabel + " — lifter→HOME (waiting)");
    }

    if ((cycleState == CYCLE_COLLECTING || cycleState == CYCLE_LOWERING) && camMoveEnabled) {
      if (cycleState == CYCLE_COLLECTING) {
        long dist = readDistanceCM();
        if (dist < 8) {
          // Too close — stop, treat as collected
          motorStop();
          addLog("CYCLE: ultrasonic stop at " + String(dist) + "cm");
        } else {
          if      (steer < -TRACK_STEER_DEAD) motorSpinLeft(TRACK_TURN_SPEED);
          else if (steer >  TRACK_STEER_DEAD) motorSpinRight(TRACK_TURN_SPEED);
          else                                motorForward(TRACK_DRIVE_SPEED);
        }
      }
    }

    server.send(200, "text/plain", "OK");
  });

  // Called by ESP32-CAM when no trash is detected — trash collected!
  server.on("/trackLost", []() {
    if (cycleState == CYCLE_COLLECTING) {
      // Trash gone from frame — assume collected
      motorStop();
      collectorStop();
      trackingTrash = false;
      camPaused     = true;   // pause CAM inference

      // Tell lifter to go UP (1580) — non-blocking
      lifterGoUp();

      cycleState       = CYCLE_LIFT_UP;
      addLog("CYCLE: trash collected — motor stop, lifter→UP, CAM paused");
    }
    server.send(200, "text/plain", "OK");
  });

  // ── CAM pause check — CAM polls this to know if it should pause ──
  server.on("/camPaused", []() {
    server.send(200, "text/plain", camPaused ? "1" : "0");
  });

  // ── Full auto cycle status ────────────────────────────────
  server.on("/cycleStatus", []() {
    String state = "idle";
    if      (cycleState == CYCLE_LOWERING)   state = "lowering";
    else if (cycleState == CYCLE_COLLECTING) state = "collecting";
    else if (cycleState == CYCLE_LIFT_UP)    state = "lifting";
    else if (cycleState == CYCLE_SETTLE)     state = "settling";
    else if (cycleState == CYCLE_SEGREGATE)  state = "sorting";
    String j = "{";
    j += "\"state\":\"" + state + "\"";
    j += ",\"label\":\"" + lastTrashLabel + "\"";
    j += ",\"camScan\":"  + String(camScanEnabled ? "true" : "false");
    j += ",\"camMove\":"  + String(camMoveEnabled ? "true" : "false");
    j += ",\"paused\":"   + String(camPaused      ? "true" : "false");
    j += "}";
    server.send(200, "application/json", j);
  });

  // ── Full auto toggle ──────────────────────────────────────
  server.on("/fullAuto", []() {
    if (server.hasArg("state")) {
      bool on = server.arg("state").toInt();
      camScanEnabled = on;
      camMoveEnabled = on;
      if (!on) {
        // stop everything
        motorStop();
        collectorStop();
        trackingTrash  = false;
        cycleState     = CYCLE_IDLE;
        camPaused      = false;
        lastTrashLabel = "";
        addLog("FULL AUTO: disabled — all stopped");
      } else {
        addLog("FULL AUTO: enabled");
      }
    }
    server.send(200, "text/plain", "OK");
  });

  // ── CAM auto-scan toggle ──────────────────────────────────
  // ?state=1 to enable, ?state=0 to disable
  server.on("/camScan", []() {
    if (server.hasArg("state")) {
      camScanEnabled = server.arg("state").toInt();
      if (!camScanEnabled) {
        // turning off — stop any in-progress tracking
        trackingTrash = false;
        trackedLabel  = "";
        motorStop();
        addLog("CAM SCAN: disabled");
      } else {
        addLog("CAM SCAN: enabled");
      }
    }
    // return current state as JSON
    String j = "{\"camScan\":" + String(camScanEnabled ? "true" : "false") + "}";
    server.send(200, "application/json", j);
  });

  // ── Dashboard data route ─────────────────────────────────
  server.on("/dashboard", []() {
    int total = countBio + countNonBio + countRecycle;
    String j = "{";
    j += "\"bio\":"      + String(countBio);
    j += ",\"nonbio\":"  + String(countNonBio);
    j += ",\"recycle\":" + String(countRecycle);
    j += ",\"total\":"   + String(total);
    j += ",\"uptime\":"  + String(millis() / 1000);
    j += ",\"heap\":"    + String(ESP.getFreeHeap());
    j += ",\"rssi\":"    + String(WiFi.RSSI());
    j += "}";
    server.send(200, "application/json", j);
  });

  // ── Reset counters ────────────────────────────────────────
  server.on("/resetCounters", []() {
    countBio = 0; countNonBio = 0; countRecycle = 0;
    addLog("Counters reset");
    server.send(200, "text/plain", "OK");
  });

  server.on("/", handleRoot);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  addLog("HTTP server started");
  Serial.println("HTTP server started");

  // Start lifter background poll task on Core 0
  lifterMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(lifterPollTask, "lifterPoll", 4096, NULL, 1, NULL, 0);
  Serial.println("Lifter poll task started");
}

// ═══════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  server.handleClient();
  updateSong();

  if (currentMode == AVOID || currentMode == FOLLOW) {
    unsigned long now = millis();
    if (now - lastAutoRun >= AUTO_INTERVAL) {
      lastAutoRun = now;
      if (currentMode == AVOID)  runAvoid();
      if (currentMode == FOLLOW) runFollow();
    }
  }

  // ── Trash separator state machine ──
  if (autoSorting && millis() - sortStartMs >= SORT_HOLD_MS) {
    autoSorting = false;
    trashSetServo(POS_HOME);
    trashSetMotor(MOTOR_STOP);
    addLog("AUTO-SORT done -> HOME + motor STOP");
  }
  if (recyclState == 1 && millis() - recyclStartMs >= RECYCLE_UP_MS) {
    trashSetMotor(MOTOR_STOP);
    recyclState   = 2;
    recyclStartMs = millis();
    addLog("RECYCLE -> UP done, pausing");
  }
  if (recyclState == 2 && millis() - recyclStartMs >= RECYCLE_PAUSE_MS) {
    trashSetMotor(MOTOR_DOWN);
    recyclState   = 3;
    recyclStartMs = millis();
    addLog("RECYCLE -> motor DOWN returning");
  }
  if (recyclState == 3 && millis() - recyclStartMs >= RECYCLE_DOWN_MS) {
    trashSetMotor(MOTOR_STOP);
    recyclState = 0;
    addLog("RECYCLE -> cycle complete, STOP");
  }

  // ── Auto-cycle state machine ──────────────────────────────
  unsigned long nowMs = millis();

  // CYCLE_LOWERING: wait for lifter to reach 0 then start collector + move
  if (cycleState == CYCLE_LOWERING) {
    lifterPollActive = true;
    if (!lifterIsMoving()) {
      lifterPollActive = false;
      collectorForward(137);
      cycleState = CYCLE_COLLECTING;
      addLog("CYCLE: lifter at 0 — collector FWD + bot moving");
    }
  }

  // CYCLE_LIFT_UP: wait for lifter to reach 1580
  if (cycleState == CYCLE_LIFT_UP) {
    lifterPollActive = true;
    if (!lifterIsMoving()) {
      lifterPollActive = false;
      lifterReachedUp  = true;
      cycleState       = CYCLE_SETTLE;
      cycleTimer       = nowMs;
      addLog("CYCLE: lifter UP reached — settling " + String(SETTLE_MS/1000) + "s");
    }
  }

  // CYCLE_SETTLE: wait 5 seconds for trash to slide down
  if (cycleState == CYCLE_SETTLE && nowMs - cycleTimer >= SETTLE_MS) {
    cycleState = CYCLE_SEGREGATE;
    addLog("CYCLE: settle done — segregating: " + lastTrashLabel);

    // Trigger segregation based on last label
    if (lastTrashLabel == "bio" && !autoSorting && recyclState == 0) {
      trashSetServo(POS_BIO);
      autoSorting = true;
      sortStartMs = nowMs;
      countBio++;
      addLog("CYCLE SEG: BIO -> servo LEFT (" + String(countBio) + " total)");
    } else if (lastTrashLabel == "nonbio" && !autoSorting && recyclState == 0) {
      trashSetServo(POS_NONBIO);
      autoSorting = true;
      sortStartMs = nowMs;
      countNonBio++;
      addLog("CYCLE SEG: NON-BIO -> servo RIGHT (" + String(countNonBio) + " total)");
    } else if (lastTrashLabel == "recycle" && recyclState == 0 && abs(trashServoUs - POS_HOME) <= 100) {
      trashSetMotor(MOTOR_UP);
      recyclState   = 1;
      recyclStartMs = nowMs;
      countRecycle++;
      addLog("CYCLE SEG: RECYCLABLE -> conveyor UP (" + String(countRecycle) + " total)");
    }
    cycleTimer = nowMs;
  }

  // CYCLE_SEGREGATE: wait for sort to finish then resume CAM
  if (cycleState == CYCLE_SEGREGATE && !autoSorting && recyclState == 0) {
    // Sort done — resume CAM and reset cycle
    cycleState     = CYCLE_IDLE;
    camPaused      = false;
    trackingTrash  = false;
    lastTrashLabel = "";
    addLog("CYCLE DONE — CAM resumed, ready for next trash");
  }
}

// ═══════════════════════════════════════════════════════════
// WEB UI
// ═══════════════════════════════════════════════════════════
void handleRoot() {
  String html = R"rawhtml(<!DOCTYPE html><html><head>
<meta name='viewport' content='width=device-width, initial-scale=1, user-scalable=no, maximum-scale=1'>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; user-select: none; -webkit-user-select: none; }
body {
  background: #1a1a1a; display: flex; flex-direction: column;
  align-items: center; justify-content: flex-start;
  min-height: 100vh; font-family: sans-serif; color: white;
  overflow-y: auto; touch-action: pan-y;
  -webkit-tap-highlight-color: transparent; padding-bottom: 20px;
}
.view-toggle { display: flex; gap: 8px; margin: 10px 0 8px; flex-wrap: wrap; justify-content: center; }
.view-btn { padding: 7px 18px; border: 2px solid #444; border-radius: 20px; background: #2a2a2a; color: #888; font-size: 12px; font-weight: bold; cursor: pointer; outline: none; transition: all 0.15s; }
.view-btn.active { border-color: #38bdf8; color: #38bdf8; background: #1e3a5f; }
#carView   { display: flex; flex-direction: column; align-items: center; width: 100%; max-width: 480px; }
#trashView { display: none; flex-direction: column; align-items: center; width: 100%; max-width: 480px; padding: 0 12px; }
#allView   { display: none; flex-direction: column; align-items: center; width: 100%; max-width: 900px; padding: 0 12px; }
#autoView  { display: none; flex-direction: column; align-items: center; width: 100%; max-width: 480px; padding: 0 12px; }
#dashView  { display: none; flex-direction: column; align-items: center; width: 100%; max-width: 480px; padding: 0 12px; }

/* ── Desktop layout ── */
@media (min-width: 768px) {
  body { padding: 20px; }
  .view-toggle { gap: 12px; margin: 16px 0 12px; }
  .view-btn { padding: 9px 22px; font-size: 13px; }
  #carView   { max-width: 560px; }
  #trashView { max-width: 560px; }
  #allView   { max-width: 1100px; }
  #autoView  { max-width: 600px; }
  #dashView  { max-width: 900px; flex-direction: row !important; flex-wrap: wrap; align-items: flex-start; gap: 12px; }
  #dashView > div { flex: 1; min-width: 280px; max-width: 100% !important; margin-bottom: 0 !important; }
  .trash-card { max-width: 100% !important; }
  #joystick-zone { width: 240px; height: 240px; }
  #joystick-knob { width: 80px; height: 80px; }
  #btnStop { width: 80px; height: 80px; font-size: 14px; }
  .mode-btn { padding: 8px 16px; font-size: 12px; }
  .gear-btn { padding: 6px 14px; font-size: 12px; }
  .tbtn { padding: 20px 8px; font-size: 13px; }
  .tb-rec { padding: 14px; font-size: 12px; }
  h1 { font-size: 18px; margin-bottom: 10px; }
}
h1 { font-size: 15px; margin-bottom: 6px; color: #888; letter-spacing: 1px; }
.mode-row { display: flex; gap: 8px; margin-bottom: 8px; }
.mode-btn { padding: 6px 12px; border: 2px solid #444; border-radius: 20px; background: #2a2a2a; color: #aaa; font-size: 11px; font-weight: bold; cursor: pointer; touch-action: none; outline: none; transition: all 0.15s; }
.mode-btn.active         { border-color: #fff;    color: #fff;    background: #3a3a3a; }
.mode-btn.avoid.active   { border-color: #e65100; color: #e65100; }
.mode-btn.follow.active  { border-color: #1565c0; color: #4fc3f7; }
.gear-row { display: flex; gap: 6px; margin-bottom: 8px; align-items: center; }
.gear-label { font-size: 11px; color: #888; font-weight: bold; margin-right: 2px; }
.gear-btn { padding: 5px 10px; border: 2px solid #444; border-radius: 12px; background: #2a2a2a; color: #aaa; font-size: 11px; font-weight: bold; cursor: pointer; touch-action: none; outline: none; transition: all 0.15s; }
.gear-btn.active { border-color: #00e676; color: #00e676; background: #1b5e20; }
.song-row { display: flex; flex-wrap: wrap; justify-content: center; gap: 5px; margin-bottom: 8px; }
.song-btn { padding: 5px 10px; border: none; border-radius: 12px; font-size: 10px; font-weight: bold; cursor: pointer; touch-action: none; outline: none; transition: filter 0.1s; color: white; }
.song-btn:active { filter: brightness(1.5); }
.s-mario { background: #e53935; } .s-rick { background: #3d5afe; } .s-imperial { background: #6a1b9a; }
.s-tetris { background: #00897b; } .s-jingle { background: #1565c0; } .s-happy { background: #f9a825; color: #222; } .s-stop { background: #555; }
#status      { font-size: 11px; color: #555; margin-bottom: 4px; min-height: 14px; }
#distDisplay { font-size: 11px; color: #ff9800; margin-bottom: 4px; min-height: 14px; }
#autoMsg     { font-size: 13px; font-weight: bold; color: #aaa; text-align: center; display: none; padding: 0 20px; margin-bottom: 6px; }
#joystick-zone { width: 200px; height: 200px; border-radius: 50%; background: #2a2a2a; border: 3px solid #444; position: relative; touch-action: none; transition: opacity 0.2s; cursor: pointer; }
#joystick-zone.disabled { opacity: 0.25; pointer-events: none; }
#joystick-knob { width: 70px; height: 70px; border-radius: 50%; background: radial-gradient(circle at 35% 35%, #888, #333); border: 2px solid #666; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); pointer-events: none; }
#btnStop { margin-top: 10px; background: #c62828; width: 70px; height: 70px; border-radius: 50%; font-size: 13px; font-weight: bold; border: none; color: white; cursor: pointer; touch-action: none; outline: none; transition: filter 0.08s; }
#btnStop:active { filter: brightness(1.5); }
.trash-card { background: #1e293b; border: 1px solid #334155; border-radius: 16px; padding: 16px; width: 100%; max-width: 420px; margin-bottom: 12px; box-shadow: 0 6px 20px rgba(0,0,0,.4); }
.trash-title { font-size: 11px; font-weight: bold; letter-spacing: 1px; color: #64748b; text-transform: uppercase; margin-bottom: 12px; display: flex; justify-content: space-between; }
.pos-track { position: relative; height: 10px; background: #334155; border-radius: 999px; margin-bottom: 6px; overflow: visible; }
.pos-thumb { position: absolute; top: 50%; width: 18px; height: 18px; background: linear-gradient(135deg,#38bdf8,#818cf8); border-radius: 50%; transform: translate(-50%,-50%); transition: left .3s ease; border: 3px solid #0f172a; box-shadow: 0 2px 6px rgba(0,0,0,.5); }
.pos-labels { display: flex; justify-content: space-between; font-size: 10px; color: #64748b; margin-bottom: 12px; }
.trash-dual { display: flex; gap: 8px; margin-bottom: 12px; }
.trash-stat { flex: 1; background: #0f172a; border-radius: 10px; padding: 10px 6px; text-align: center; }
.trash-val { font-size: 18px; font-weight: bold; background: linear-gradient(90deg,#38bdf8,#818cf8); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
.trash-lbl { font-size: 9px; color: #64748b; margin-top: 3px; }
.trash-bins { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 7px; margin-bottom: 8px; }
.tbtn { padding: 18px 4px; font-size: 12px; font-weight: bold; border-radius: 10px; cursor: pointer; outline: none; border: 2px solid; transition: filter 0.15s; text-align: center; }
.tbtn:active { filter: brightness(1.5); }
.tb-bio  { border-color: #16a34a; background: #14532d; color: #86efac; }
.tb-home { border-color: #0369a1; background: #0c4a6e; color: #7dd3fc; }
.tb-non  { border-color: #7c3aed; background: #4c1d95; color: #c4b5fd; }
.tb-rec  { width: 100%; padding: 13px; border: 2px solid #0891b2; border-radius: 10px; background: #164e63; color: #67e8f9; font-weight: bold; cursor: pointer; outline: none; margin-bottom: 8px; transition: filter .15s, opacity .15s; }
.tb-rec:active { filter: brightness(1.5); }
.tb-rec:disabled { opacity: .35; cursor: not-allowed; }
.divider { border: none; border-top: 1px solid #334155; margin: 10px 0; }
.tb-sort { width: 100%; padding: 11px; border: none; border-radius: 10px; font-weight: bold; cursor: pointer; outline: none; background: linear-gradient(90deg,#f59e0b,#ef4444); color: #fff; transition: opacity .15s; }
.tb-sort:disabled { opacity: .35; cursor: not-allowed; }
.tb-camscan { width: 100%; padding: 11px; border: 2px solid #334155; border-radius: 10px; font-weight: bold; cursor: pointer; outline: none; background: #1e293b; color: #64748b; margin-bottom: 8px; transition: all .2s; }
.tb-camscan.on { border-color: #22c55e; background: #14532d; color: #86efac; }
.collector-row { display: flex; gap: 8px; margin-top: 8px; }
.col-btn { flex: 1; padding: 14px 4px; font-size: 13px; font-weight: bold; border-radius: 10px; cursor: pointer; outline: none; border: 2px solid; transition: filter .15s; text-align: center; }
.col-btn:active { filter: brightness(1.5); }
.col-fwd  { border-color: #0891b2; background: #164e63; color: #67e8f9; }
.col-stop { border-color: #475569; background: #1e293b; color: #94a3b8; }
.col-bwd  { border-color: #7c3aed; background: #4c1d95; color: #c4b5fd; }
.col-fwd.on  { background: #0891b2; color: #fff; }
.col-bwd.on  { background: #7c3aed; color: #fff; }
.col-speed { width: 100%; margin-top: 6px; accent-color: #38bdf8; }
.trash-msg { text-align: center; font-size: 11px; color: #64748b; min-height: 16px; margin-top: 6px; }
.badge { display: inline-block; background: #f59e0b; color: #0f172a; font-weight: bold; font-size: 10px; padding: 2px 7px; border-radius: 999px; }
.log-box { background: #020617; border: 1px solid #1e3a5f; border-radius: 10px; padding: 10px 12px; font-family: 'Courier New', monospace; font-size: 11px; color: #38bdf8; max-height: 160px; overflow-y: auto; }
.log-box p { margin: 1px 0; line-height: 1.5; }
.log-ok { color: #86efac; } .log-err { color: #f87171; }
</style>
<script>
var active = false;
var lastSentX = 999, lastSentY = 999, lastSendTime = 0;
var THROTTLE_MS = 50, DEADZONE = 10;
var xhr = null, distTimer = null, currentMode = 'MANUAL';

function sendGet(url) {
  var x = new XMLHttpRequest(); x.open('GET', url, true); x.timeout = 300; x.send(); return x;
}
function setMode(mode) {
  currentMode = mode; sendGet('/MODE_' + mode);
  document.querySelectorAll('.mode-btn').forEach(function(b){ b.classList.remove('active'); });
  document.getElementById('btn' + mode).classList.add('active');
  var joy = document.getElementById('joystick-zone');
  var msg = document.getElementById('autoMsg');
  if (mode === 'MANUAL') {
    joy.classList.remove('disabled'); msg.style.display = 'none'; stopDistPoll();
  } else {
    joy.classList.add('disabled'); msg.style.display = 'block';
    msg.textContent = mode === 'AVOID' ? '🚧 Obstacle Avoidance — Running' : '🎯 Obstacle Follow — Running';
    startDistPoll();
  }
}
function setGear(g) {
  sendGet('/gear?g=' + g);
  document.querySelectorAll('.gear-btn').forEach(function(b){ b.classList.remove('active'); });
  document.getElementById('btnGear' + g).classList.add('active');
}
function startDistPoll() {
  if (distTimer) return;
  distTimer = setInterval(function() {
    var x = new XMLHttpRequest(); x.open('GET','/dist',true); x.timeout=200;
    x.onreadystatechange = function(){ if(x.readyState===4&&x.status===200) document.getElementById('distDisplay').textContent='📡 '+x.responseText+' cm'; };
    x.send();
  }, 300);
}
function stopDistPoll() {
  if (distTimer){ clearInterval(distTimer); distTimer=null; }
  document.getElementById('distDisplay').textContent='';
}
function applyDeadzone(val) {
  if (Math.abs(val) < DEADZONE) return 0;
  return val > 0 ? Math.round((val-DEADZONE)/(100-DEADZONE)*100) : Math.round((val+DEADZONE)/(100-DEADZONE)*100);
}
function sendJoy(nx, ny) {
  var now=Date.now(), rx=Math.round(nx), ry=Math.round(ny);
  if (rx===lastSentX && ry===lastSentY) return;
  if (now-lastSendTime < THROTTLE_MS) return;
  lastSentX=rx; lastSentY=ry; lastSendTime=now;
  if (xhr){ xhr.abort(); xhr=null; }
  xhr=new XMLHttpRequest(); xhr.open('GET','/joy?x='+rx+'&y='+ry,true); xhr.timeout=200; xhr.send();
  document.getElementById('status').textContent = (rx===0&&ry===0)?'':'X: '+rx+'  Y: '+ry;
}
function resetKnob() {
  document.getElementById('joystick-knob').style.transform='translate(-50%, -50%)';
  lastSentX=999; lastSentY=999; lastSendTime=0; sendJoy(0,0);
  document.getElementById('status').textContent='';
}
function moveKnob(px, py) {
  var zone = document.getElementById('joystick-zone');
  var knob = document.getElementById('joystick-knob');
  var r = zone.getBoundingClientRect();
  var knobR = knob.offsetWidth / 2;
  var cx = r.left + r.width / 2, cy = r.top + r.height / 2;
  var dx = px - cx, dy = py - cy;
  var dist = Math.sqrt(dx * dx + dy * dy);
  var maxD = (r.width / 2) - knobR;
  
  if (dist > maxD) {
    dx = (dx / dist) * maxD;
    dy = (dy / dist) * maxD;
  }
  
  knob.style.transform = 'translate(calc(-50% + ' + dx + 'px), calc(-50% + ' + dy + 'px))';
  sendJoy(applyDeadzone((dx / maxD) * 100), applyDeadzone(-(dy / maxD) * 100));
}
function playSong(s){ sendGet('/SONG_'+s); }

window.onload=function(){
  var zone=document.getElementById('joystick-zone');
  function handleStart(e){
    if(currentMode!=='MANUAL') return; if(e.preventDefault) e.preventDefault(); active=true;
    var cx=e.clientX||(e.touches&&e.touches[0].clientX),cy=e.clientY||(e.touches&&e.touches[0].clientY);
    if(e.pointerId&&zone.setPointerCapture){try{zone.setPointerCapture(e.pointerId);}catch(err){}}
    moveKnob(cx,cy);
  }
  function handleMove(e){if(!active)return;if(e.preventDefault)e.preventDefault();var cx=e.clientX||(e.touches&&e.touches[0].clientX),cy=e.clientY||(e.touches&&e.touches[0].clientY);moveKnob(cx,cy);}
  function handleEnd(e){if(!active)return;if(e.preventDefault)e.preventDefault();active=false;resetKnob();}
  zone.addEventListener('pointerdown',handleStart,{passive:false});
  window.addEventListener('pointermove',handleMove,{passive:false});
  window.addEventListener('pointerup',handleEnd,{passive:false});
  window.addEventListener('pointercancel',handleEnd,{passive:false});
  zone.addEventListener('touchstart',handleStart,{passive:false});
  window.addEventListener('touchmove',handleMove,{passive:false});
  window.addEventListener('touchend',handleEnd,{passive:false});
  window.addEventListener('touchcancel',handleEnd,{passive:false});
  document.getElementById('btnStop').addEventListener('pointerdown',function(e){
    if(e.preventDefault)e.preventDefault();
    if(currentMode!=='MANUAL'){setMode('MANUAL');}else{active=false;resetKnob();sendGet('/STOP');}
  });
  document.getElementById('btnMANUAL').classList.add('active');
  document.getElementById('btnGear4').classList.add('active');
  // Prime trash status so thumb/labels show correctly even before trash view is opened
  fetch('/trashStatus').then(function(r){return r.json();}).then(trashUpdate).catch(function(){});
};

/* ── View toggle ── */
function setView(v){
  var car=document.getElementById('carView'),tr=document.getElementById('trashView'),al=document.getElementById('allView'),au=document.getElementById('autoView'),da=document.getElementById('dashView');
  document.getElementById('viewCar').classList.toggle('active',v==='car');
  document.getElementById('viewTrash').classList.toggle('active',v==='trash');
  document.getElementById('viewAll').classList.toggle('active',v==='all');
  document.getElementById('viewAuto').classList.toggle('active',v==='auto');
  document.getElementById('viewDash').classList.toggle('active',v==='dash');
  car.style.display=(v==='car')?'flex':'none';
  tr.style.display=(v==='trash')?'flex':'none';
  al.style.display=(v==='all')?'flex':'none';
  au.style.display=(v==='auto')?'flex':'none';
  da.style.display=(v==='dash')?'flex':'none';
  if(v==='trash'||v==='all'){trashPoll();}else{if(trashTimer){clearInterval(trashTimer);trashTimer=null;}}
  if(v==='all'){ pollLifterBatt(); initJoy2(); }
  if(v==='auto'){ startAutoPoll(); document.getElementById('camStream').src='http://10.60.114.138:81/stream?' + Date.now(); }
  else { stopAutoPoll(); document.getElementById('camStream').src=''; }
  if(v==='dash'){ startDashPoll(); } else { stopDashPoll(); }
}

// ── Full Auto JS ──────────────────────────────────────────
var fullAutoOn    = false;
var autoPollTimer = null;

function toggleFullAuto() {
  fullAutoOn = !fullAutoOn;
  fetch('/fullAuto?state=' + (fullAutoOn ? 1 : 0));
  updateFullAutoBtn();
}

function fullAutoStop() {
  fullAutoOn = false;
  fetch('/fullAuto?state=0');
  updateFullAutoBtn();
  // also stop bot immediately
  fetch('/STOP');
}

function updateFullAutoBtn() {
  var btn = document.getElementById('btnFullAuto');
  if (!btn) return;
  if (fullAutoOn) {
    btn.textContent = '\u23F9 STOP FULL AUTO';
    btn.style.background    = '#14532d';
    btn.style.borderColor   = '#22c55e';
    btn.style.color         = '#86efac';
  } else {
    btn.textContent = '\u25B6 START FULL AUTO';
    btn.style.background    = '#0f172a';
    btn.style.borderColor   = '#334155';
    btn.style.color         = '#64748b';
  }
}

var cycleColors = {
  idle:       { badge: '&#9679; IDLE',        color: '#64748b', bg: '#1e293b' },
  lowering:   { badge: '&#9660; LOWERING',    color: '#38bdf8', bg: '#0c4a6e' },
  collecting: { badge: '&#128663; COLLECTING', color: '#fcd34d', bg: '#422006' },
  lifting:    { badge: '&#9650; LIFTING',      color: '#86efac', bg: '#14532d' },
  settling:   { badge: '&#8987; SETTLING',     color: '#67e8f9', bg: '#164e63' },
  sorting:    { badge: '&#9851; SORTING',      color: '#c4b5fd', bg: '#2e1065' }
};

function updateCycleUI(d) {
  var state = d.state || 'idle';
  var c = cycleColors[state] || cycleColors.idle;
  var badge = document.getElementById('cycleStatusBadge');
  if (badge) { badge.innerHTML = c.badge; badge.style.color = c.color; badge.style.background = c.bg; badge.style.borderColor = c.color; }

  var lbl = document.getElementById('lastLabel');
  if (lbl) lbl.textContent = d.label || '--';

  var steps = { lowering:'csLower', collecting:'csCollect', lifting:'csLift', settling:'csSettle', sorting:'csSort' };
  Object.keys(steps).forEach(function(k) {
    var el = document.getElementById(steps[k]);
    if (!el) return;
    if (k === state) {
      el.style.background  = c.bg;
      el.style.color       = c.color;
      el.style.borderColor = c.color;
    } else {
      el.style.background  = '#1e293b';
      el.style.color       = '#475569';
      el.style.borderColor = '#334155';
    }
  });

  if (d.camScan && d.camMove && !fullAutoOn) { fullAutoOn = true;  updateFullAutoBtn(); }
  if (!d.camScan && !d.camMove && fullAutoOn){ fullAutoOn = false; updateFullAutoBtn(); }
}

function startAutoPoll() {
  if (autoPollTimer) return;
  autoPollTimer = setInterval(function() {
    fetch('/cycleStatus').then(function(r){ return r.json(); }).then(updateCycleUI).catch(function(){});
  }, 600);
}
function stopAutoPoll() {
  if (autoPollTimer) { clearInterval(autoPollTimer); autoPollTimer = null; }
}

// ── Dashboard JS ──────────────────────────────────────────
var dashPollTimer = null;

function startDashPoll() {
  if (dashPollTimer) return;
  updateDash();
  dashPollTimer = setInterval(updateDash, 2000);
}
function stopDashPoll() {
  if (dashPollTimer) { clearInterval(dashPollTimer); dashPollTimer = null; }
}

function updateDash() {
  // Bot dashboard data
  fetch('/dashboard').then(function(r){ return r.json(); }).then(function(d) {
    document.getElementById('dBio').textContent     = d.bio;
    document.getElementById('dNonBio').textContent  = d.nonbio;
    document.getElementById('dRecycle').textContent = d.recycle;
    document.getElementById('dTotal').textContent   = d.total;

    // Uptime
    var s = d.uptime, h = Math.floor(s/3600), m = Math.floor((s%3600)/60), sec = s%60;
    document.getElementById('dUptime').textContent =
      (h>0?h+'h ':'') + (m>0?m+'m ':'') + sec+'s';

    // Heap
    document.getElementById('dHeap').textContent =
      Math.round(d.heap/1024) + ' KB';

    // WiFi RSSI
    var rssi = d.rssi;
    var rssiTxt = rssi + ' dBm';
    var rssiColor = rssi > -60 ? '#86efac' : rssi > -75 ? '#fcd34d' : '#f87171';
    var rssiEl = document.getElementById('dRSSI');
    rssiEl.textContent = rssiTxt;
    rssiEl.style.color = rssiColor;

    // Bot battery estimate (based on runtime — rough)
    // 4x 18650 = ~14.8V, assume full at start, rough runtime estimate
    var runMins = d.uptime / 60;
    var botPct  = Math.max(0, Math.min(100, 100 - (runMins * 0.5)));
    document.getElementById('dBotBatt').textContent = Math.round(botPct) + '%';
    document.getElementById('dBotBattBar').style.width = botPct + '%';
    document.getElementById('dBotBattBar').style.background =
      botPct > 50 ? 'linear-gradient(90deg,#22c55e,#86efac)' :
      botPct > 25 ? 'linear-gradient(90deg,#f59e0b,#fcd34d)' :
                    'linear-gradient(90deg,#ef4444,#f87171)';
  }).catch(function(){});

  // Lifter battery from ESP8266
  fetch('http://10.60.114.130/battery').then(function(r){ return r.text(); }).then(function(v) {
    var volts = parseFloat(v);
    // 3x 18650 = ~12.6V full, ~9V empty
    var pct = Math.max(0, Math.min(100, ((volts - 9.0) / (12.6 - 9.0)) * 100));
    document.getElementById('dLifterBatt').textContent = volts.toFixed(2) + 'V (' + Math.round(pct) + '%)';
    document.getElementById('dLifterBattBar').style.width = pct + '%';
    document.getElementById('dLifterBattBar').style.background =
      pct > 50 ? 'linear-gradient(90deg,#f59e0b,#fcd34d)' :
      pct > 25 ? 'linear-gradient(90deg,#f97316,#fb923c)' :
                 'linear-gradient(90deg,#ef4444,#f87171)';
  }).catch(function(){
    document.getElementById('dLifterBatt').textContent = 'offline';
  });
}

function resetCounters() {
  fetch('/resetCounters').then(function(){ updateDash(); });
}

var solarPowerMap = { CHARGING: '5.2 W', STANDBY: '0 W', LOW: '1.1 W' };
function setSolarStatus(status) {
  var el  = document.getElementById('dSolarStatus');
  var pw  = document.getElementById('dSolarPower');
  var col = status === 'CHARGING' ? '#fcd34d' : status === 'LOW' ? '#f87171' : '#64748b';
  el.textContent  = status;
  el.style.color  = col;
  pw.textContent  = solarPowerMap[status] || '-- W';
  pw.style.color  = col;
}

var joy2init=false;
function initJoy2(){
  if(joy2init)return; joy2init=true;
  var zone=document.getElementById('joystick-zone2');
  var knob=document.getElementById('joystick-knob2');
  var active2=false;
  function move2(px,py){
    var r=zone.getBoundingClientRect(),knobR=knob.offsetWidth/2;
    var cx=r.left+r.width/2,cy=r.top+r.height/2;
    var dx=px-cx,dy=py-cy,dist=Math.sqrt(dx*dx+dy*dy),maxD=(r.width/2)-knobR;
    if(dist>maxD){dx=(dx/dist)*maxD;dy=(dy/dist)*maxD;}
    knob.style.transform='translate(calc(-50% + '+dx+'px), calc(-50% + '+dy+'px))';
    sendJoy(applyDeadzone((dx/maxD)*100),applyDeadzone(-(dy/maxD)*100));
  }
  function reset2(){active2=false;knob.style.transform='translate(-50%,-50%)';sendJoy(0,0);}
  zone.addEventListener('pointerdown',function(e){if(currentMode!=='MANUAL')return;e.preventDefault();active2=true;if(zone.setPointerCapture)try{zone.setPointerCapture(e.pointerId);}catch(ex){}move2(e.clientX,e.clientY);},{passive:false});
  window.addEventListener('pointermove',function(e){if(!active2)return;e.preventDefault();move2(e.clientX,e.clientY);},{passive:false});
  window.addEventListener('pointerup',function(e){if(!active2)return;reset2();},{passive:false});
  zone.addEventListener('touchstart',function(e){if(currentMode!=='MANUAL')return;e.preventDefault();active2=true;move2(e.touches[0].clientX,e.touches[0].clientY);},{passive:false});
  window.addEventListener('touchmove',function(e){if(!active2)return;e.preventDefault();move2(e.touches[0].clientX,e.touches[0].clientY);},{passive:false});
  window.addEventListener('touchend',function(e){if(!active2)return;reset2();},{passive:false});
}

/* ── Trash JS ── */
var trashTimer=null,logLines=[];
var thumbEl,tServoEl,tMotorEl,tRecEl,tSortEl,tMsgEl,tLogBox;
function usToPercent(us){return(us-500)/(2500-500)*100;}
function trashEls(){
  if(!thumbEl){
    thumbEl=document.getElementById('tThumb');tServoEl=document.getElementById('tServo');
    tMotorEl=document.getElementById('tMotor');tRecEl=document.getElementById('tRec');
    tSortEl=document.getElementById('tSort');tMsgEl=document.getElementById('tMsg');
    tLogBox=document.getElementById('tLogBox');
  }
}
function trashUpdate(d){
  trashEls();
  thumbEl.style.left=usToPercent(d.servo)+'%';
  tServoEl.textContent=d.sname;tMotorEl.textContent=d.mname;
  tSortEl.disabled=d.sorting;
  if(d.recycling){
    tRecEl.disabled=true;
    var lbl=d.rstate===1?'&#8679; UP&hellip;':d.rstate===2?'&#9646; pause&hellip;':'&#8681; DOWN&hellip;';
    tMsgEl.innerHTML='&#9851; Cycle: <b>'+lbl+'</b> <span class="badge">RUNNING</span>';
  }else{
    tRecEl.disabled=!d.atHome;tRecEl.title=d.atHome?'':'Press HOME first';
    if(!d.sorting)tMsgEl.textContent='';
  }
  if(d.sorting){tMsgEl.innerHTML='auto-sorting&hellip; <span class="badge">ACTIVE</span>';}
  if(d.log&&d.log.length){
    logLines=d.log;
    tLogBox.innerHTML=logLines.map(function(l){
      var c=l.indexOf('ERROR')>=0?'log-err':(l.indexOf('OK')>=0||l.indexOf('HOME')>=0)?'log-ok':'';
      return '<p class="'+c+'">&rsaquo; '+l+'</p>';
    }).join('');
  }
}
function trashPoll(){
  if(trashTimer)clearInterval(trashTimer);
  var poll=function(){var x=new XMLHttpRequest();x.open('GET','/trashStatus',true);x.timeout=400;x.onreadystatechange=function(){if(x.readyState===4&&x.status===200){try{trashUpdate(JSON.parse(x.responseText));}catch(e){}}};x.send();};
  poll();trashTimer=setInterval(poll,400);
}
function trashGet(url,msg){
  trashEls();
  if(tMsgEl)tMsgEl.textContent=msg||'';
  fetch(url).then(function(r){return r.json();}).then(trashUpdate).catch(function(){if(tMsgEl)tMsgEl.textContent='connection error';});
}
function trashRecycle(){
  trashEls();
  if(tRecEl&&tRecEl.disabled)return;
  tRecEl.disabled=true;
  tMsgEl.innerHTML='&#9851; Cycle: <b>&#8679; UP&hellip;</b> <span class="badge">RUNNING</span>';
  fetch('/doRecyclable').then(function(r){return r.json();}).then(function(d){
    trashUpdate(d);
    if(d.blocked){tMsgEl.innerHTML='&#9888; Servo not at HOME!';tRecEl.disabled=false;return;}
    var poll=setInterval(function(){fetch('/trashStatus').then(function(r){return r.json();}).then(function(d){trashUpdate(d);if(!d.recycling){clearInterval(poll);tMsgEl.textContent='Recycle cycle complete';}});},300);
  }).catch(function(){tMsgEl.textContent='connection error';tRecEl.disabled=false;});
}
function trashAutoSort(){
  trashEls();
  tSortEl.disabled=true;
  tMsgEl.innerHTML='auto-sorting&hellip; <span class="badge">ACTIVE</span>';
  fetch('/sort').then(function(r){return r.json();}).then(trashUpdate).catch(function(){tMsgEl.textContent='error';tSortEl.disabled=false;});
  var poll=setInterval(function(){fetch('/trashStatus').then(function(r){return r.json();}).then(function(d){trashUpdate(d);if(!d.sorting){clearInterval(poll);tMsgEl.textContent='sort done - back at HOME';}});},400);
}

var collectorState = 'stop';
function toggleCollector(dir) {
  var spd = document.getElementById('colSpeed') ? document.getElementById('colSpeed').value : 200;
  if (collectorState === dir) {
    collectorState = 'stop';
    sendGet('/collector?dir=stop&spd=' + spd);
    ['colFwdBtn','colFwdBtn2'].forEach(function(id){var e=document.getElementById(id);if(e)e.classList.remove('on');});
    ['colBwdBtn','colBwdBtn2'].forEach(function(id){var e=document.getElementById(id);if(e)e.classList.remove('on');});
  } else {
    collectorState = dir;
    sendGet('/collector?dir=' + dir + '&spd=' + spd);
    ['colFwdBtn','colFwdBtn2'].forEach(function(id){var e=document.getElementById(id);if(e)e.classList.toggle('on',dir==='fwd');});
    ['colBwdBtn','colBwdBtn2'].forEach(function(id){var e=document.getElementById(id);if(e)e.classList.toggle('on',dir==='bwd');});
  }
}
function stopCollector() {
  var spd = document.getElementById('colSpeed') ? document.getElementById('colSpeed').value : 200;
  collectorState = 'stop';
  sendGet('/collector?dir=stop&spd=' + spd);
  ['colFwdBtn','colFwdBtn2'].forEach(function(id){var e=document.getElementById(id);if(e)e.classList.remove('on');});
  ['colBwdBtn','colBwdBtn2'].forEach(function(id){var e=document.getElementById(id);if(e)e.classList.remove('on');});
}
function collectorCmd(dir) {
  var spd = document.getElementById('colSpeed') ? document.getElementById('colSpeed').value : 200;
  sendGet('/collector?dir=' + dir + '&spd=' + spd);
}

var LIFTER_IP = '10.60.114.130';
function lifterCmd(endpoint) {
  fetch('http://' + LIFTER_IP + endpoint).then(function(r){ return r.json(); }).then(updateLifterUI).catch(function(){});
}
function lifterMoveTo() {
  var s = document.getElementById('lifterSteps').value || 0;
  fetch('http://' + LIFTER_IP + '/moveTo?steps=' + s).then(function(r){ return r.json(); }).then(updateLifterUI);
}
function lifterMoveTo2() {
  var s = document.getElementById('lifterSteps2').value || 0;
  document.getElementById('lifterSteps').value = s;
  fetch('http://' + LIFTER_IP + '/moveTo?steps=' + s).then(function(r){ return r.json(); }).then(updateLifterUI);
}
function updateLifterUI(d) {
  if (!d) return;
  var pct = Math.min(100, Math.max(0, (d.pos / 1200) * 100));
  var th  = document.getElementById('lifterThumb');
  var th2 = document.getElementById('lifterThumb2');
  var p   = document.getElementById('lifterPos');
  var p2  = document.getElementById('lifterPos2');
  var b   = document.getElementById('lifterBatt');
  var b2  = document.getElementById('lifterBatt2');
  if(th)  th.style.left  = pct + '%';
  if(th2) th2.style.left = pct + '%';
  if(p)   p.textContent  = d.pos;
  if(p2)  p2.textContent = d.pos;
  if(b)   b.innerHTML    = '&#128267; ' + d.batt + 'V';
  if(b2)  b2.innerHTML   = '&#128267; ' + d.batt + 'V';
}
function pollLifterBatt() {
  fetch('http://' + LIFTER_IP + '/status')
    .then(function(r){ return r.json(); })
    .then(updateLifterUI)
    .catch(function(){
      var txt = '&#128267; offline';
      var el  = document.getElementById('lifterBatt');
      var el2 = document.getElementById('lifterBatt2');
      if(el)  el.innerHTML  = txt;
      if(el2) el2.innerHTML = txt;
    });
}
setInterval(pollLifterBatt, 1000);

var camScanOn = false;
function toggleCamScan() {
  camScanOn = !camScanOn;
  var btn  = document.getElementById('btnCamScan');
  var btn2 = document.getElementById('btnCamScan2');
  fetch('/camScan?state=' + (camScanOn ? 1 : 0))
    .then(function(r){ return r.json(); })
    .then(function(d){
      camScanOn = d.camScan;
      var txt = camScanOn ? '\uD83D\uDCF7 CAM Scan: ON' : '\uD83D\uDCF7 CAM Scan: OFF';
      if(btn)  { btn.textContent  = txt; btn.classList.toggle('on',  camScanOn); }
      if(btn2) { btn2.textContent = txt; btn2.classList.toggle('on', camScanOn); }
    });
}

var camMoveOn = false;
function toggleCamMove() {
  camMoveOn = !camMoveOn;
  var btn  = document.getElementById('btnCamMove');
  var btn2 = document.getElementById('btnCamMove2');
  fetch('/camMove?state=' + (camMoveOn ? 1 : 0))
    .then(function(r){ return r.json(); })
    .then(function(d){
      camMoveOn = d.camMove;
      var txt = camMoveOn ? '\uD83D\uDE97 CAM Move: ON' : '\uD83D\uDE97 CAM Move: OFF';
      if(btn)  { btn.textContent  = txt; btn.classList.toggle('on',  camMoveOn); }
      if(btn2) { btn2.textContent = txt; btn2.classList.toggle('on', camMoveOn); }
    });
}
</script>
</head><body>

<div class='view-toggle'>
  <button id='viewCar'   class='view-btn active' onclick='setView("car")'>&#x1F697; Car</button>
  <button id='viewTrash' class='view-btn'         onclick='setView("trash")'>&#9851; Trash</button>
  <button id='viewAll'   class='view-btn'         onclick='setView("all")'>&#9889; All</button>
  <button id='viewAuto'  class='view-btn'         onclick='setView("auto")'>&#129302; Auto</button>
  <button id='viewDash'  class='view-btn'         onclick='setView("dash")'>&#128200; Dash</button>
</div>

<div id='carView'>
<h1>OMNI DRIVE</h1>
<div class='mode-row'>
  <button id='btnMANUAL' class='mode-btn manual' onclick='setMode("MANUAL")'>&#x1F579; Manual</button>
  <button id='btnAVOID'  class='mode-btn avoid'  onclick='setMode("AVOID")'>&#x1F6A7; Avoid</button>
  <button id='btnFOLLOW' class='mode-btn follow' onclick='setMode("FOLLOW")'>&#x1F3AF; Follow</button>
</div>
<div class='gear-row'>
  <span class='gear-label'>GEAR:</span>
  <button id='btnGear1' class='gear-btn' onclick='setGear(1)'>1 (Slow)</button>
  <button id='btnGear2' class='gear-btn' onclick='setGear(2)'>2</button>
  <button id='btnGear3' class='gear-btn' onclick='setGear(3)'>3</button>
  <button id='btnGear4' class='gear-btn active' onclick='setGear(4)'>4 (Fast)</button>
</div>
<div class='song-row'>
  <button class='song-btn s-mario'    onclick='playSong("MARIO")'>&#x1F344; Mario</button>
  <button class='song-btn s-rick'     onclick='playSong("RICK")'>&#x1F3B5; Rick Roll</button>
  <button class='song-btn s-imperial' onclick='playSong("IMPERIAL")'>&#x1FA90; Imperial</button>
  <button class='song-btn s-tetris'   onclick='playSong("TETRIS")'>&#x1F3AE; Tetris</button>
  <button class='song-btn s-jingle'   onclick='playSong("JINGLE")'>&#x1F514; Jingle</button>
  <button class='song-btn s-happy'    onclick='playSong("HAPPY")'>&#x1F382; Birthday</button>
  <button class='song-btn s-stop'     onclick='playSong("STOP")'>&#x23F9; Stop</button>
</div>
<div id='distDisplay'></div>
<div id='status'></div>
<div id='autoMsg'></div>
<div id='joystick-zone'><div id='joystick-knob'></div></div>
<button id='btnStop'>STOP</button>
</div>

<div id='trashView'>
  <div class='trash-card'>
    <div class='trash-title'><span>Bin Selection</span></div>
    <div class='pos-track'><div class='pos-thumb' id='tThumb'></div></div>
    <div class='pos-labels'>
      <span>BIO<br>789&#181;s</span>
      <span style='text-align:center'>HOME<br>1680&#181;s</span>
      <span style='text-align:right'>NON-BIO<br>2500&#181;s</span>
    </div>
    <div class='trash-dual'>
      <div class='trash-stat'><div class='trash-val' id='tServo'>HOME</div><div class='trash-lbl'>sorting servo</div></div>
      <div class='trash-stat'><div class='trash-val' id='tMotor'>STOP</div><div class='trash-lbl'>conveyor motor</div></div>
    </div>
    <div class='trash-bins'>
      <button class='tbtn tb-bio'  onclick='trashGet("/doBio","BIO...")'>&#127807; BIO<br><small>servo LEFT</small></button>
      <button class='tbtn tb-home' onclick='trashGet("/doHome","going HOME...")'>&#8962; HOME<br><small>center+stop</small></button>
      <button class='tbtn tb-non'  onclick='trashGet("/doNonBio","NON-BIO...")'>&#9851; NON-BIO<br><small>servo RIGHT</small></button>
    </div>
    <button class='tb-rec' id='tRec' onclick='trashRecycle()'>&#9851; RECYCLABLE &mdash; auto UP &rarr; STOP &rarr; DOWN &rarr; STOP</button>
    <div class='divider'></div>
    <button id='btnCamScan' class='tb-camscan' onclick='toggleCamScan()'>&#128247; CAM Auto-Scan: OFF</button>
    <button class='tb-sort' id='tSort' onclick='trashAutoSort()'>&#9654; Auto Sort <small>(BIO &rarr; hold 1.5s &rarr; HOME)</small></button>
    <div class='trash-msg' id='tMsg'></div>
  </div>

  <div class='trash-card'>
    <div class='trash-title'><span>&#9881; Collector Motor</span></div>
    <div class='collector-row'>
      <button id='colFwdBtn' class='col-btn col-fwd'  onclick='toggleCollector("fwd")'>&#9650; FWD</button>
      <button class='col-btn col-stop' onclick='stopCollector()'>&#9646; STOP</button>
      <button id='colBwdBtn' class='col-btn col-bwd'  onclick='toggleCollector("bwd")'>&#9660; BWD</button>
    </div>
    <input type='range' class='col-speed' id='colSpeed' min='80' max='255' value='200' oninput='document.getElementById("colSpeedVal").textContent=this.value'>
    <div style='text-align:center;font-size:11px;color:#64748b;margin-top:3px'>Speed: <span id='colSpeedVal'>200</span></div>
  </div>
  <div class='trash-card'>
    <div class='trash-title'>
      <span>&#128196; Debug Log</span>
      <span style='cursor:pointer;color:#ef4444;font-size:10px' onclick='tLogBox.innerHTML="<p style=color:#334155>cleared</p>"'>clear</span>
    </div>
    <div class='log-box' id='tLogBox'><p style='color:#334155'>waiting&hellip;</p></div>
  </div>

  <div class='trash-card'>
    <div class='trash-title'>
      <span>&#9883; Trash Lifter</span>
      <span id='lifterBatt' style='color:#4ecca3;font-size:11px'>&#128267; --V</span>
    </div>
    <!-- Position track -->
    <div style='position:relative;height:10px;background:#334155;border-radius:999px;margin-bottom:6px;'>
      <div id='lifterThumb' style='position:absolute;top:50%;width:18px;height:18px;background:linear-gradient(135deg,#38bdf8,#818cf8);border-radius:50%;transform:translate(-50%,-50%);left:0%;transition:left .3s;border:3px solid #0f172a;'></div>
    </div>
    <div style='display:flex;justify-content:space-between;font-size:10px;color:#64748b;margin-bottom:8px;'>
      <span>HOME<br>0</span><span style='text-align:center'>MID<br>400</span><span style='text-align:right'>UP<br>800</span>
    </div>
    <!-- Presets -->
    <div class='collector-row' style='margin-bottom:6px;'>
      <button class='col-btn col-stop' onclick='lifterCmd("/goHome")' style='flex:1;'>&#8962; HOME</button>
      <button class='col-btn' style='flex:1;border-color:#7c3aed;background:#4c1d95;color:#c4b5fd;' onclick='lifterCmd("/goMid")'>&#9654; MID</button>
      <button class='col-btn col-fwd'  onclick='lifterCmd("/goUp")'  style='flex:1;'>&#9650; UP</button>
    </div>
    <!-- Custom steps -->
    <div style='display:flex;gap:6px;margin-bottom:6px;'>
      <input type='number' id='lifterSteps' min='0' max='1200' value='0' placeholder='0-1200 steps'
        style='flex:1;background:#0f172a;border:1px solid #334155;border-radius:8px;color:#38bdf8;font-size:13px;padding:7px;text-align:center;outline:none;'>
      <button onclick='lifterMoveTo()' style='padding:7px 14px;background:linear-gradient(90deg,#38bdf8,#818cf8);border:none;border-radius:8px;color:#0f172a;font-weight:bold;cursor:pointer;'>GO</button>
    </div>
    <!-- Continuous -->
    <div class='collector-row'>
      <button class='col-btn col-fwd'  onclick='lifterCmd("/start_cw")'>&#8635; CW</button>
      <button class='col-btn col-stop' onclick='lifterCmd("/stop")'>&#9646; STOP</button>
      <button class='col-btn col-bwd'  onclick='lifterCmd("/start_ccw")'>&#8634; CCW</button>
    </div>
    <div style='text-align:center;font-size:10px;color:#475569;margin-top:6px;'>&#128204; <span id='lifterPos'>0</span> steps &nbsp;|&nbsp; 10.60.114.130</div>
  </div>
</div>

<div id='allView' style='display:none;flex-direction:column;align-items:center;width:100%;padding:4px 8px;'>

  <!-- TOP ROW: Movement left | Segregation right -->
  <div style='display:flex;gap:6px;width:100%;margin-bottom:6px;'>

    <!-- LEFT: Movement -->
    <div style='flex:1;background:#1e293b;border:1px solid #334155;border-radius:12px;padding:8px;display:flex;flex-direction:column;align-items:center;'>
      <div style='font-size:10px;font-weight:bold;color:#64748b;letter-spacing:1px;margin-bottom:4px;'>&#x1F697; MOVEMENT</div>
      <!-- Mode buttons -->
      <div style='display:flex;gap:4px;margin-bottom:4px;width:100%;'>
        <button id='btnMANUAL2' class='mode-btn manual active' onclick='setMode("MANUAL")' style='flex:1;padding:4px 2px;font-size:9px;'>&#x1F579; Man</button>
        <button id='btnAVOID2'  class='mode-btn avoid'         onclick='setMode("AVOID")'  style='flex:1;padding:4px 2px;font-size:9px;'>&#x1F6A7; Avoid</button>
        <button id='btnFOLLOW2' class='mode-btn follow'        onclick='setMode("FOLLOW")' style='flex:1;padding:4px 2px;font-size:9px;'>&#x1F3AF; Follow</button>
      </div>
      <!-- Gear -->
      <div style='display:flex;gap:3px;margin-bottom:6px;align-items:center;'>
        <span style='font-size:9px;color:#888;font-weight:bold;'>G:</span>
        <button id='btnGear1b' class='gear-btn' onclick='setGear(1)' style='padding:3px 7px;font-size:9px;'>1</button>
        <button id='btnGear2b' class='gear-btn' onclick='setGear(2)' style='padding:3px 7px;font-size:9px;'>2</button>
        <button id='btnGear3b' class='gear-btn' onclick='setGear(3)' style='padding:3px 7px;font-size:9px;'>3</button>
        <button id='btnGear4b' class='gear-btn active' onclick='setGear(4)' style='padding:3px 7px;font-size:9px;'>4</button>
      </div>
      <!-- Joystick -->
      <div id='joystick-zone2' style='width:130px;height:130px;border-radius:50%;background:#2a2a2a;border:3px solid #444;position:relative;touch-action:none;cursor:pointer;margin-bottom:5px;'>
        <div id='joystick-knob2' style='width:46px;height:46px;border-radius:50%;background:radial-gradient(circle at 35% 35%,#888,#333);border:2px solid #666;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none'></div>
      </div>
      <button onclick='sendGet("/STOP")' style='background:#c62828;width:50px;height:50px;border-radius:50%;font-size:10px;font-weight:bold;border:none;color:white;cursor:pointer;'>STOP</button>
    </div>

    <!-- RIGHT: Segregation + CAM -->
    <div style='flex:1;background:#1e293b;border:1px solid #334155;border-radius:12px;padding:8px;display:flex;flex-direction:column;'>
      <div style='font-size:10px;font-weight:bold;color:#64748b;letter-spacing:1px;margin-bottom:6px;text-align:center;'>&#9851; SEGREGATION</div>
      <!-- Bin buttons -->
      <div style='display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px;margin-bottom:5px;'>
        <button class='tbtn tb-bio'  onclick='trashGet("/doBio","BIO...")'    style='padding:10px 2px;font-size:10px;'>&#127807;<br><small>BIO</small></button>
        <button class='tbtn tb-home' onclick='trashGet("/doHome","HOME...")'  style='padding:10px 2px;font-size:10px;'>&#8962;<br><small>HOME</small></button>
        <button class='tbtn tb-non'  onclick='trashGet("/doNonBio","NON-BIO...")' style='padding:10px 2px;font-size:10px;'>&#9851;<br><small>NON</small></button>
      </div>
      <button id='tRec2' class='tb-rec' onclick='trashRecycle()' style='padding:8px;font-size:10px;margin-bottom:5px;'>&#9851; RECYCLABLE</button>
      <!-- CAM toggles -->
      <button id='btnCamScan2' class='tb-camscan' onclick='toggleCamScan()' style='margin-bottom:4px;padding:7px;font-size:10px;'>&#128247; CAM Scan: OFF</button>
      <button id='btnCamMove2' class='tb-cammove' onclick='toggleCamMove()' style='margin-bottom:0;padding:7px;font-size:10px;'>&#128663; CAM Move: OFF</button>
    </div>

  </div>

  <!-- BOTTOM ROW: Collector left | Lifter right -->
  <div style='display:flex;gap:6px;width:100%;'>

    <!-- Collector -->
    <div style='flex:1;background:#1e293b;border:1px solid #334155;border-radius:12px;padding:8px;'>
      <div style='font-size:10px;font-weight:bold;color:#64748b;letter-spacing:1px;margin-bottom:5px;text-align:center;'>&#9881; COLLECTOR</div>
      <div style='display:flex;gap:4px;margin-bottom:5px;'>
        <button id='colFwdBtn2' class='col-btn col-fwd' onclick='toggleCollector("fwd")' style='flex:1;padding:10px 2px;font-size:11px;'>&#9650; FWD</button>
        <button class='col-btn col-stop' onclick='stopCollector()' style='flex:1;padding:10px 2px;font-size:11px;'>&#9646;</button>
        <button id='colBwdBtn2' class='col-btn col-bwd' onclick='toggleCollector("bwd")' style='flex:1;padding:10px 2px;font-size:11px;'>&#9660; BWD</button>
      </div>
      <input type='range' class='col-speed' id='colSpeed2' min='80' max='255' value='200' oninput='document.getElementById("colSpeedVal2").textContent=this.value;document.getElementById("colSpeed").value=this.value;'>
      <div style='text-align:center;font-size:9px;color:#64748b;'>Spd: <span id='colSpeedVal2'>200</span></div>
    </div>

    <!-- Lifter -->
    <div style='flex:1;background:#1e293b;border:1px solid #334155;border-radius:12px;padding:8px;'>
      <div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;'>
        <span style='font-size:10px;font-weight:bold;color:#64748b;letter-spacing:1px;'>&#9883; LIFTER</span>
        <span id='lifterBatt2' style='color:#4ecca3;font-size:10px;'>&#128267; --V</span>
      </div>
      <div style='position:relative;height:8px;background:#334155;border-radius:999px;margin-bottom:4px;'>
        <div id='lifterThumb2' style='position:absolute;top:50%;width:14px;height:14px;background:linear-gradient(135deg,#38bdf8,#818cf8);border-radius:50%;transform:translate(-50%,-50%);left:0%;transition:left .3s;border:2px solid #0f172a;'></div>
      </div>
      <div style='display:flex;justify-content:space-between;font-size:9px;color:#64748b;margin-bottom:4px;'><span>HOME 0</span><span style='text-align:right'>UP 1580</span></div>
      <div style='display:flex;gap:3px;margin-bottom:4px;'>
        <button class='col-btn col-stop' onclick='lifterCmd("/goHome")' style='flex:1;padding:6px 2px;font-size:9px;'>&#8962; HOME</button>
        <button class='col-btn col-fwd'  onclick='lifterCmd("/goUp")'  style='flex:1;padding:6px 2px;font-size:9px;'>&#9650; UP</button>
      </div>
      <div style='display:flex;gap:4px;margin-bottom:4px;'>
        <input type='number' id='lifterSteps2' min='0' max='99999' value='0' placeholder='steps'
          style='flex:1;background:#0f172a;border:1px solid #334155;border-radius:6px;color:#38bdf8;font-size:11px;padding:5px;text-align:center;outline:none;'>
        <button onclick='lifterMoveTo2()' style='padding:5px 10px;background:linear-gradient(90deg,#38bdf8,#818cf8);border:none;border-radius:6px;color:#0f172a;font-weight:bold;cursor:pointer;font-size:10px;'>GO</button>
      </div>
      <div style='display:flex;gap:3px;'>
        <button class='col-btn col-fwd'  onclick='lifterCmd("/start_cw")'  style='flex:1;padding:7px 2px;font-size:10px;'>&#8635; CW</button>
        <button class='col-btn col-stop' onclick='lifterCmd("/stop")'       style='flex:1;padding:7px 2px;font-size:10px;'>&#9646;</button>
        <button class='col-btn col-bwd'  onclick='lifterCmd("/start_ccw")' style='flex:1;padding:7px 2px;font-size:10px;'>&#8634; CCW</button>
      </div>
      <div style='text-align:center;font-size:9px;color:#475569;margin-top:3px;'><span id='lifterPos2'>0</span> steps</div>
    </div>

  </div>

</div>

<!-- ⚡ FULL AUTO VIEW -->
<div id='autoView' style='display:none;flex-direction:column;align-items:center;width:100%;padding:8px 12px;'>

  <!-- Status card -->
  <div style='background:#1e293b;border:1px solid #334155;border-radius:14px;padding:14px;width:100%;max-width:420px;margin-bottom:10px;'>
    <div style='font-size:11px;font-weight:bold;color:#64748b;letter-spacing:1px;margin-bottom:10px;text-align:center;'>&#129302; FULL AUTO MODE</div>

    <!-- Big toggle -->
    <button id='btnFullAuto' onclick='toggleFullAuto()'
      style='width:100%;padding:16px;border-radius:12px;font-size:16px;font-weight:bold;border:2px solid #334155;background:#0f172a;color:#64748b;cursor:pointer;transition:all .2s;margin-bottom:10px;'>
      &#9654; START FULL AUTO
    </button>

    <!-- Cycle status -->
    <div style='background:#0f172a;border-radius:10px;padding:12px;margin-bottom:10px;'>
      <div style='font-size:10px;color:#64748b;margin-bottom:6px;'>CYCLE STATUS</div>
      <div id='cycleStatusBadge' style='display:inline-block;padding:4px 12px;border-radius:999px;font-size:12px;font-weight:bold;background:#1e293b;color:#64748b;border:1px solid #334155;margin-bottom:8px;'>&#9679; IDLE</div>
      <div style='display:flex;gap:6px;flex-wrap:wrap;'>
        <div id='csLower'   style='flex:1;min-width:60px;padding:6px 4px;border-radius:8px;text-align:center;font-size:9px;background:#1e293b;color:#475569;border:1px solid #334155;'>&#9660; LOWER</div>
        <div id='csCollect' style='flex:1;min-width:60px;padding:6px 4px;border-radius:8px;text-align:center;font-size:9px;background:#1e293b;color:#475569;border:1px solid #334155;'>&#128663; COLLECT</div>
        <div id='csLift'    style='flex:1;min-width:60px;padding:6px 4px;border-radius:8px;text-align:center;font-size:9px;background:#1e293b;color:#475569;border:1px solid #334155;'>&#9650; LIFT UP</div>
        <div id='csSettle'  style='flex:1;min-width:60px;padding:6px 4px;border-radius:8px;text-align:center;font-size:9px;background:#1e293b;color:#475569;border:1px solid #334155;'>&#8987; SETTLE</div>
        <div id='csSort'    style='flex:1;min-width:60px;padding:6px 4px;border-radius:8px;text-align:center;font-size:9px;background:#1e293b;color:#475569;border:1px solid #334155;'>&#9851; SORT</div>
      </div>
    </div>

    <!-- Last detection -->
    <div style='background:#0f172a;border-radius:10px;padding:10px;margin-bottom:10px;display:flex;justify-content:space-between;align-items:center;'>
      <span style='font-size:10px;color:#64748b;'>Last Detected</span>
      <span id='lastLabel' style='font-size:14px;font-weight:bold;color:#38bdf8;'>--</span>
    </div>

    <!-- CAM stream embed -->
    <div style='background:#0f172a;border-radius:10px;overflow:hidden;margin-bottom:10px;'>
      <div style='font-size:10px;color:#64748b;padding:6px 10px;'>&#128247; CAM Stream — 10.60.114.138</div>
      <img id='camStream' src='' style='width:100%;display:block;border-radius:0 0 10px 10px;min-height:180px;background:#000;object-fit:contain;'
        onerror='this.style.opacity=0.3;this.alt="Stream offline"'>
    </div>

    <!-- Emergency STOP -->
    <button onclick='fullAutoStop()'
      style='width:100%;padding:14px;border-radius:12px;font-size:15px;font-weight:bold;border:none;background:#c62828;color:white;cursor:pointer;'>
      &#9632; EMERGENCY STOP
    </button>
  </div>

</div>

<!-- 📊 DASHBOARD VIEW -->
<div id='dashView' style='display:none;flex-direction:column;align-items:center;width:100%;padding:8px 12px;'>

  <!-- Trash counters -->
  <div style='background:#1e293b;border:1px solid #334155;border-radius:14px;padding:14px;width:100%;max-width:420px;margin-bottom:10px;'>
    <div style='font-size:11px;font-weight:bold;color:#64748b;letter-spacing:1px;margin-bottom:12px;display:flex;justify-content:space-between;'>
      <span>&#9851; TRASH SEGREGATED</span>
      <span onclick='resetCounters()' style='color:#ef4444;font-size:10px;cursor:pointer;'>&#8635; Reset</span>
    </div>
    <div style='display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:10px;'>
      <div style='background:#0f172a;border-radius:10px;padding:10px;text-align:center;border:1px solid #14532d;'>
        <div style='font-size:28px;font-weight:bold;color:#86efac;' id='dBio'>0</div>
        <div style='font-size:9px;color:#64748b;margin-top:3px;'>&#127807; BIO</div>
      </div>
      <div style='background:#0f172a;border-radius:10px;padding:10px;text-align:center;border:1px solid #4c1d95;'>
        <div style='font-size:28px;font-weight:bold;color:#c4b5fd;' id='dNonBio'>0</div>
        <div style='font-size:9px;color:#64748b;margin-top:3px;'>&#9851; NON-BIO</div>
      </div>
      <div style='background:#0f172a;border-radius:10px;padding:10px;text-align:center;border:1px solid #164e63;'>
        <div style='font-size:28px;font-weight:bold;color:#67e8f9;' id='dRecycle'>0</div>
        <div style='font-size:9px;color:#64748b;margin-top:3px;'>&#9842; RECYCLE</div>
      </div>
    </div>
    <div style='background:#0f172a;border-radius:10px;padding:10px;text-align:center;'>
      <div style='font-size:32px;font-weight:bold;background:linear-gradient(90deg,#38bdf8,#818cf8);-webkit-background-clip:text;-webkit-text-fill-color:transparent;' id='dTotal'>0</div>
      <div style='font-size:10px;color:#64748b;'>TOTAL SEGREGATED</div>
    </div>
  </div>

  <!-- Battery health -->
  <div style='background:#1e293b;border:1px solid #334155;border-radius:14px;padding:14px;width:100%;max-width:420px;margin-bottom:10px;'>
    <div style='font-size:11px;font-weight:bold;color:#64748b;letter-spacing:1px;margin-bottom:12px;'>&#128267; BATTERY STATUS</div>
    <!-- Bot battery (estimated from ESP heap/runtime) -->
    <div style='margin-bottom:10px;'>
      <div style='display:flex;justify-content:space-between;font-size:10px;color:#94a3b8;margin-bottom:4px;'>
        <span>&#x1F697; Bot Battery (4x 18650)</span>
        <span id='dBotBatt'>--</span>
      </div>
      <div style='background:#334155;border-radius:999px;height:8px;'>
        <div id='dBotBattBar' style='background:linear-gradient(90deg,#22c55e,#86efac);height:8px;border-radius:999px;width:0%;transition:width .5s;'></div>
      </div>
    </div>
    <!-- Lifter battery (from ESP8266 ADC) -->
    <div style='margin-bottom:4px;'>
      <div style='display:flex;justify-content:space-between;font-size:10px;color:#94a3b8;margin-bottom:4px;'>
        <span>&#9883; Lifter Battery (3x 18650)</span>
        <span id='dLifterBatt'>--</span>
      </div>
      <div style='background:#334155;border-radius:999px;height:8px;'>
        <div id='dLifterBattBar' style='background:linear-gradient(90deg,#f59e0b,#fcd34d);height:8px;border-radius:999px;width:0%;transition:width .5s;'></div>
      </div>
    </div>
  </div>

  <!-- Solar status (UI only) -->
  <div style='background:#1e293b;border:1px solid #334155;border-radius:14px;padding:14px;width:100%;max-width:420px;margin-bottom:10px;'>
    <div style='font-size:11px;font-weight:bold;color:#64748b;letter-spacing:1px;margin-bottom:12px;display:flex;justify-content:space-between;'>
      <span>&#9728; SOLAR STATUS</span>
      <span style='font-size:9px;color:#475569;'>UI only</span>
    </div>
    <div style='display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:10px;'>
      <div style='background:#0f172a;border-radius:10px;padding:10px;text-align:center;'>
        <div style='font-size:20px;font-weight:bold;color:#fcd34d;'>&#9728;</div>
        <div style='font-size:11px;font-weight:bold;color:#fcd34d;margin:4px 0;' id='dSolarStatus'>CHARGING</div>
        <div style='font-size:9px;color:#64748b;'>panel status</div>
      </div>
      <div style='background:#0f172a;border-radius:10px;padding:10px;text-align:center;'>
        <div style='font-size:20px;font-weight:bold;color:#86efac;'>&#9889;</div>
        <div style='font-size:11px;font-weight:bold;color:#86efac;margin:4px 0;' id='dSolarPower'>-- W</div>
        <div style='font-size:9px;color:#64748b;'>estimated power</div>
      </div>
    </div>
    <!-- Solar toggle (UI only) -->
    <div style='display:flex;gap:8px;'>
      <button onclick='setSolarStatus("CHARGING")' style='flex:1;padding:8px;border-radius:8px;border:1px solid #fcd34d;background:#422006;color:#fcd34d;font-size:10px;font-weight:bold;cursor:pointer;'>&#9728; Charging</button>
      <button onclick='setSolarStatus("STANDBY")' style='flex:1;padding:8px;border-radius:8px;border:1px solid #334155;background:#1e293b;color:#64748b;font-size:10px;font-weight:bold;cursor:pointer;'>&#9711; Standby</button>
      <button onclick='setSolarStatus("LOW")' style='flex:1;padding:8px;border-radius:8px;border:1px solid #ef4444;background:#450a0a;color:#f87171;font-size:10px;font-weight:bold;cursor:pointer;'>&#9888; Low</button>
    </div>
  </div>

  <!-- System info -->
  <div style='background:#1e293b;border:1px solid #334155;border-radius:14px;padding:14px;width:100%;max-width:420px;margin-bottom:10px;'>
    <div style='font-size:11px;font-weight:bold;color:#64748b;letter-spacing:1px;margin-bottom:10px;'>&#128187; SYSTEM INFO</div>
    <div style='display:grid;grid-template-columns:1fr 1fr;gap:6px;'>
      <div style='background:#0f172a;border-radius:8px;padding:8px;'>
        <div style='font-size:14px;font-weight:bold;color:#38bdf8;' id='dUptime'>--</div>
        <div style='font-size:9px;color:#64748b;'>uptime</div>
      </div>
      <div style='background:#0f172a;border-radius:8px;padding:8px;'>
        <div style='font-size:14px;font-weight:bold;color:#38bdf8;' id='dHeap'>--</div>
        <div style='font-size:9px;color:#64748b;'>free heap</div>
      </div>
      <div style='background:#0f172a;border-radius:8px;padding:8px;'>
        <div style='font-size:14px;font-weight:bold;color:#38bdf8;' id='dRSSI'>--</div>
        <div style='font-size:9px;color:#64748b;'>WiFi signal</div>
      </div>
      <div style='background:#0f172a;border-radius:8px;padding:8px;'>
        <div style='font-size:14px;font-weight:bold;color:#38bdf8;'>10.60.114.42</div>
        <div style='font-size:9px;color:#64748b;'>bot IP</div>
      </div>
    </div>
  </div>

</div>

</body></html>)rawhtml";

  server.send(200, "text/html", html);
}
