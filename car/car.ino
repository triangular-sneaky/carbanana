// 2x TB6612 + ESP32  --  CAR (receiver)
// Receives dual-joystick input over ESP-NOW and drives 4 MECANUM wheels
// holonomically (forward/back, strafe, rotate). Variable speed via LEDC PWM on
// the TB6612 PWMA/PWMB pins (Approach A: slow-decay for good low-speed control).
//
// Motor layout (viewed from above):
//    FL (M1) --- FR (M2)
//    BL (M3) --- BR (M4)
//
// Mecanum mix (signed speed per wheel):
//    FL = y + x + r      FR = y - x - r
//    BL = y - x + r      BR = y + x - r
//   y = forward, x = strafe, r = rotation.

#include <esp_now.h>
#include <WiFi.h>

// ---- Motor direction pins (IN1/IN2) ----------------------------------------
// Driver 1
const int M1_IN1 = 18;  // AIN1 FL
const int M1_IN2 = 19;  // AIN2
const int M2_IN1 = 21;  // BIN1 FR
const int M2_IN2 = 22;  // BIN2
// Driver 2
const int M3_IN1 = 25;  // AIN1 BL
const int M3_IN2 = 26;  // AIN2
const int M4_IN1 = 27;  // BIN1 BR
const int M4_IN2 = 33;  // BIN2

// ---- Motor speed pins (PWMA/PWMB) ------------------------------------------
// Rewired: PWMA/PWMB jumpers removed, PWM pads wired to these GPIOs.
const int M1_PWM = 4;   // Driver1 PWMA -> FL
const int M2_PWM = 5;   // Driver1 PWMB -> FR
const int M3_PWM = 16;  // Driver2 PWMA -> BL
const int M4_PWM = 17;  // Driver2 PWMB -> BR

// ---- Solenoid --------------------------------------------------------------
const int SOLENOID_PIN = 23;               // digital out (ADC2 pins are fine as outputs)
const unsigned long SOL_OFF_MS = 1000;     // off duration
const unsigned long SOL_ON_MS  = 100;      // on pulse
const bool SOL_ACTIVE_HIGH = true;         // set false if the driver is active-low

// ---- PWM config ------------------------------------------------------------
const int PWM_FREQ = 20000;  // 20 kHz, above audible range
const int PWM_RES  = 8;      // 8-bit duty (0..255)
const int MAX_DUTY = 255;
const int MIN_DUTY = 60;     // motor won't turn below ~this; nonzero commands lift to it

// ---- Wireless packet (must match RC side) ----------------------------------
typedef struct {
  int16_t x;    // strafe
  int16_t y;    // forward
  int16_t r;    // rotation
  uint8_t btn;  // 1 = button pressed, 0 = released
} JoystickPacket;

volatile int16_t rxX = 0;
volatile int16_t rxY = 0;
volatile int16_t rxR = 0;
volatile uint8_t rxBtn = 0;
volatile unsigned long lastRxMs = 0;

// Deadzone is handled (calibrated) on the RC side; inputs arrive already zeroed
// inside the band, so the car only shapes and mixes.
const float AXIS_SCALE = 2048.0;        // maps centered reading to ~ -1..1
const unsigned long FAILSAFE_MS = 500;  // stop if no packet within this window

// Set to 1 for serial debug (received inputs + computed wheel duties), 0 to disable.
#define DEBUG 1
const unsigned long DEBUG_INTERVAL_MS = 200;  // throttle prints so serial stays readable

// ---- Motor control ---------------------------------------------------------
// cmd is signed -MAX_DUTY..+MAX_DUTY: sign = direction, magnitude = speed.
void setMotor(int in1, int in2, int pwmPin, int cmd) {
  if (cmd >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    cmd = -cmd;
  }
  if (cmd > MAX_DUTY) cmd = MAX_DUTY;
  ledcWrite(pwmPin, cmd);
}

void stopAll() {
  ledcWrite(M1_PWM, 0);
  ledcWrite(M2_PWM, 0);
  ledcWrite(M3_PWM, 0);
  ledcWrite(M4_PWM, 0);
}

// Non-blocking solenoid loop: SOL_OFF_MS off, then SOL_ON_MS on, repeating.
// DISABLED for now (not called from loop) — kept for when we want the sequence
// back. The button-triggered one-off click below is used instead.
void solenoidUpdate() {
  static unsigned long last = 0;
  static bool on = false;
  unsigned long interval = on ? SOL_ON_MS : SOL_OFF_MS;
  if (millis() - last >= interval) {
    last = millis();
    on = !on;
    digitalWrite(SOLENOID_PIN, (on == SOL_ACTIVE_HIGH) ? HIGH : LOW);
  }
}

// One-off solenoid click: fires a single SOL_ON_MS pulse, non-blocking.
// solenoidClick() starts the pulse; solenoidOneShotUpdate() ends it on time.
unsigned long solClickOffAt = 0;  // millis() deadline to turn off; 0 = idle

void solenoidClick() {
  digitalWrite(SOLENOID_PIN, SOL_ACTIVE_HIGH ? HIGH : LOW);
  solClickOffAt = millis() + SOL_ON_MS;
}

void solenoidOneShotUpdate() {
  if (solClickOffAt != 0 && millis() >= solClickOffAt) {
    digitalWrite(SOLENOID_PIN, SOL_ACTIVE_HIGH ? LOW : HIGH);
    solClickOffAt = 0;
  }
}

// ---- Input shaping ---------------------------------------------------------
// Centered ADC reading -> normalized -1..1 (deadzone already applied on RC).
float normAxis(int v) {
  float f = v / AXIS_SCALE;
  if (f > 1.0f) f = 1.0f;
  if (f < -1.0f) f = -1.0f;
  return f;
}

// Cubic expo curve, sign-preserving (odd power keeps the sign).
// Flatter near center than squared -> finer control at low input.
float expo(float f) {
  return f * f * f;
}

// Map a normalized wheel value (-1..1) to signed duty. Lifts any nonzero command
// up to the mechanical floor (MIN_DUTY) so the wheel actually turns, then scales
// the remaining range up to MAX_DUTY. Zero stays zero (stop).
int shapeDuty(float v) {
  if (v > -0.0001f && v < 0.0001f) return 0;
  float mag = fabsf(v);
  if (mag > 1.0f) mag = 1.0f;
  int duty = MIN_DUTY + (int)(mag * (MAX_DUTY - MIN_DUTY) + 0.5f);
  return v < 0 ? -duty : duty;
}

// ---- Serial command interface ----------------------------------------------
// Type into the Serial Monitor (newline-terminated):
//   x,y        e.g. "1,1" or "0.5,-0.5"  -> drive at normalized x,y (r=0)
//   x,y,r      e.g. "0,0,1"              -> include rotation
//   ...,Ns     e.g. "1,1,2s" or "0,0,1,3s" -> run the command for N sec, then stop
//   rc                                    -> hand control back to the RC link
//   stop / s                              -> serial mode, all axes 0 (halt)
// Values are normalized -1..1, LINEAR (no expo), clamped. Without a duration the
// command persists until you change it or type "rc"; the RC failsafe is
// suppressed in serial mode, so use "stop" or "rc" to halt.
enum DriveMode { MODE_RC, MODE_SERIAL };
DriveMode driveMode = MODE_RC;
float cmdX = 0, cmdY = 0, cmdR = 0;      // normalized -1..1, used in serial mode
unsigned long cmdExpireAt = 0;           // millis() deadline; 0 = hold indefinitely

char serBuf[32];
uint8_t serLen = 0;

float clampUnit(float v) {
  if (v > 1.0f) return 1.0f;
  if (v < -1.0f) return -1.0f;
  return v;
}

void handleSerialLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;  // skip leading space
  if (line[0] == '\0') return;

  if (strcasecmp(line, "rc") == 0) {
    driveMode = MODE_RC;
    Serial.println("-> RC mode");
    return;
  }
  if (strcasecmp(line, "stop") == 0 || strcasecmp(line, "s") == 0) {
    driveMode = MODE_SERIAL;
    cmdX = cmdY = cmdR = 0;
    cmdExpireAt = 0;
    Serial.println("-> SERIAL stop (0,0,0)");
    return;
  }

  // Parse "x,y" or "x,y,r", plus an optional duration token containing 's'
  // (e.g. "2s", "1.5s") = run this command for N seconds then stop.
  float vals[3] = {0, 0, 0};
  int n = 0;
  float durSec = 0;
  bool haveDur = false;
  char *tok = strtok(line, ", ");
  while (tok) {
    if (strchr(tok, 's') || strchr(tok, 'S')) {
      durSec = atof(tok);   // atof stops at the 's'
      haveDur = true;
    } else if (n < 3) {
      vals[n++] = atof(tok);
    }
    tok = strtok(NULL, ", ");
  }
  if (n >= 2) {
    cmdX = clampUnit(vals[0]);
    cmdY = clampUnit(vals[1]);
    cmdR = (n >= 3) ? clampUnit(vals[2]) : 0.0f;
    driveMode = MODE_SERIAL;
    cmdExpireAt = (haveDur && durSec > 0) ? millis() + (unsigned long)(durSec * 1000.0f) : 0;
    Serial.print("-> SERIAL x="); Serial.print(cmdX);
    Serial.print(" y="); Serial.print(cmdY);
    Serial.print(" r="); Serial.print(cmdR);
    if (cmdExpireAt) { Serial.print(" for "); Serial.print(durSec); Serial.print("s"); }
    Serial.println();
  } else {
    Serial.println("? use: x,y[,r] [Ns]  |  rc  |  stop");
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serLen > 0) {
        serBuf[serLen] = '\0';
        handleSerialLine(serBuf);
        serLen = 0;
      }
    } else if (serLen < sizeof(serBuf) - 1) {
      serBuf[serLen++] = c;
    }
  }
}

// ---- ESP-NOW receive callback ----------------------------------------------
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(JoystickPacket)) return;
  JoystickPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  rxX = pkt.x;
  rxY = pkt.y;
  rxR = pkt.r;
  rxBtn = pkt.btn;
  lastRxMs = millis();
}

void setup() {
  // Solenoid off first thing (safe at boot).
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, SOL_ACTIVE_HIGH ? LOW : HIGH);

  // Direction pins
  pinMode(M1_IN1, OUTPUT);  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);  pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT);  pinMode(M4_IN2, OUTPUT);

  // Speed pins (LEDC PWM, core 3.x pin-based API)
  ledcAttach(M1_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(M2_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(M3_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(M4_PWM, PWM_FREQ, PWM_RES);
  stopAll();

  // Serial is always up so the command interface works even with DEBUG off.
  Serial.begin(115200);
  delay(200);
  Serial.println("CAR starting...  (serial cmds: x,y | x,y,r | rc | stop)");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    // ESP-NOW failed to start; leave motors stopped.
#if DEBUG
    Serial.println("ESP-NOW init FAILED");
#endif
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  // Repeating solenoid sequence is DISABLED for now (function kept, not called).
  //   solenoidUpdate();
  // Instead: finish any in-progress one-off click (non-blocking).
  solenoidOneShotUpdate();

  // Read any pending serial command (may switch mode / set cmdX,Y,R).
  pollSerial();

  float x, y, r;  // normalized axis values fed into the mecanum mix

  if (driveMode == MODE_SERIAL) {
    // Timed command: stop when the duration elapses.
    if (cmdExpireAt != 0 && millis() >= cmdExpireAt) {
      cmdX = cmdY = cmdR = 0;
      cmdExpireAt = 0;
      Serial.println("-> SERIAL command expired, stop");
    }
    // Direct linear command; no expo, no failsafe (holds until changed/"rc").
    x = cmdX;
    y = cmdY;
    r = cmdR;
  } else {
    // RC mode. Failsafe: stop if we've lost the transmitter.
    if (millis() - lastRxMs > FAILSAFE_MS) {
      stopAll();
      delay(10);
      return;
    }
    // Normalize + expo each axis.
    x = expo(normAxis(rxX));  // strafe
    y = expo(normAxis(rxY));  // forward
    r = expo(normAxis(rxR));  // rotation
  }

  // One-off solenoid click on the button's press edge (RC packets only).
  static uint8_t prevBtn = 0;
  uint8_t btn = rxBtn;
  if (btn && !prevBtn) solenoidClick();
  prevBtn = btn;

  // Mecanum mix. Flip a wheel's signs here if it runs the wrong way.
  float fl = y + x + r;
  float fr = y - x - r;
  float bl = y - x + r;
  float br = y + x - r;

  // Normalize so no wheel exceeds full scale (preserves the motion direction).
  float m = 1.0f;
  m = max(m, fabsf(fl));
  m = max(m, fabsf(fr));
  m = max(m, fabsf(bl));
  m = max(m, fabsf(br));

  int dFL = shapeDuty(fl / m);
  int dFR = shapeDuty(fr / m);
  int dBL = shapeDuty(bl / m);
  int dBR = shapeDuty(br / m);

  setMotor(M1_IN1, M1_IN2, M1_PWM, dFL);  // FL
  setMotor(M2_IN1, M2_IN2, M2_PWM, dFR);  // FR
  setMotor(M3_IN1, M3_IN2, M3_PWM, dBL);  // BL
  setMotor(M4_IN1, M4_IN2, M4_PWM, dBR);  // BR

#if DEBUG
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg >= DEBUG_INTERVAL_MS) {
    lastDbg = millis();
    Serial.print(driveMode == MODE_SERIAL ? "[SER] " : "[RC]  ");
    Serial.print("in x="); Serial.print(rxX);
    Serial.print(" y=");   Serial.print(rxY);
    Serial.print(" r=");   Serial.print(rxR);
    Serial.print(" btn="); Serial.print(rxBtn);
    Serial.print("  duty FL="); Serial.print(dFL);
    Serial.print(" FR=");       Serial.print(dFR);
    Serial.print(" BL=");       Serial.print(dBL);
    Serial.print(" BR=");       Serial.println(dBR);
  }
#endif

  delay(10);
}
