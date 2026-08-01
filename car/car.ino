// 2x TB6612 + ESP32  --  CAR (receiver)
// Receives joystick X/Y over ESP-NOW and drives 4 motors (differential/tank mix).
//
// Motor layout (viewed from above):
//    FL (M1) --- FR (M2)
//    BL (M3) --- BR (M4)
// Left side  = FL + BL,  Right side = FR + BR.

#include <esp_now.h>
#include <WiFi.h>

// ---- Motor pins ------------------------------------------------------------
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

// ---- Wireless packet (must match RC side) ----------------------------------
typedef struct {
  int16_t x;  // steering, centered at 0  (roughly -2048 .. +2048)
  int16_t y;  // throttle, centered at 0  (roughly -2048 .. +2048)
} JoystickPacket;

volatile int16_t rxX = 0;
volatile int16_t rxY = 0;
volatile unsigned long lastRxMs = 0;

const int DEADZONE = 300;       // ignore small stick movement / noise
const unsigned long FAILSAFE_MS = 500;  // stop if no packet within this window

// ---- Low level motor control ----------------------------------------------
void stopMotor(int in1, int in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void forwardMotor(int in1, int in2) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void backwardMotor(int in1, int in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

// Drive one side (two motors) by a signed command with deadzone.
void driveSide(int in1a, int in2a, int in1b, int in2b, int cmd) {
  if (cmd > DEADZONE) {
    forwardMotor(in1a, in2a);
    forwardMotor(in1b, in2b);
  } else if (cmd < -DEADZONE) {
    backwardMotor(in1a, in2a);
    backwardMotor(in1b, in2b);
  } else {
    stopMotor(in1a, in2a);
    stopMotor(in1b, in2b);
  }
}

void stopAll() {
  stopMotor(M1_IN1, M1_IN2);
  stopMotor(M2_IN1, M2_IN2);
  stopMotor(M3_IN1, M3_IN2);
  stopMotor(M4_IN1, M4_IN2);
}

// ---- ESP-NOW receive callback ----------------------------------------------
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(JoystickPacket)) return;
  JoystickPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  rxX = pkt.x;
  rxY = pkt.y;
  lastRxMs = millis();
}

void setup() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  stopAll();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    // ESP-NOW failed to start; leave motors stopped.
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  // Failsafe: stop if we've lost the transmitter.
  if (millis() - lastRxMs > FAILSAFE_MS) {
    stopAll();
    delay(10);
    return;
  }

  int x = rxX;
  int y = rxY;

  // Differential (tank) mix: forward = both sides same, turn = sides differ.
  int left  = y + x;
  int right = y - x;

  driveSide(M1_IN1, M1_IN2, M3_IN1, M3_IN2, left);   // FL + BL
  driveSide(M2_IN1, M2_IN2, M4_IN1, M4_IN2, right);  // FR + BR

  delay(10);
}
