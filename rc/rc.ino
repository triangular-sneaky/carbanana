// ESP32  --  RC (transmitter)
// Dual analog joystick -> ESP-NOW -> car (mecanum, holonomic).
//   Left stick : X = strafe (VP/GPIO36), Y = forward/back (VN/GPIO39)
//   Right stick: X = rotation (GPIO34);  Y = spare (GPIO35)
// Both sticks must be on ADC1 pins because ESP-NOW/WiFi disables ADC2.

#include <esp_now.h>
#include <WiFi.h>

// ---- Joystick pins (all ADC1) ----------------------------------------------
const int JOY1_X = 36;  // VP  - strafe
const int JOY1_Y = 39;  // VN  - forward/back
const int JOY2_X = 34;  //     - rotation
const int JOY2_Y = 35;  //     - spare / reserved

// ---- Button ----------------------------------------------------------------
// One button to GPIO13 <-> GND, using the internal pull-up: idle reads HIGH,
// pressed reads LOW (active-low). If yours is a module with its own pull-up,
// wire its S pin here and VCC to 3V3 (never 5V). RC-local for now: just logged.
const int BTN_PIN = 13;
const unsigned long BTN_DEBOUNCE_MS = 25;
bool btnStable = HIGH;          // last debounced level
bool btnLastRead = HIGH;        // last raw read
unsigned long btnLastChange = 0;

// Stick centers AND deadzones are measured at boot from the resting sticks.
// Keep both sticks centered and still during power-up / reset.
int center1X = 2048, center1Y = 2048;
int center2X = 2048, center2Y = 2048;

// Per-axis deadzone, computed from observed jitter (not hardcoded).
int dz1X = 8, dz1Y = 8, dz2X = 8;

const float DZ_FACTOR = 1.2;  // deadzone = observed jitter * this
const int   DZ_MIN    = 6;    // floor: some zone even if a pot reads rock-steady
const int   DZ_MAX    = 300;  // cap: guards against a stick moved during calibration

// ---- Wireless packet (must match car side) ---------------------------------
typedef struct {
  int16_t x;    // strafe   (left stick X, centered)
  int16_t y;    // forward  (left stick Y, centered)
  int16_t r;    // rotation (right stick X, centered)
  uint8_t btn;  // 1 = button pressed, 0 = released (active-low read inverted)
} JoystickPacket;

// Broadcast to any ESP-NOW peer listening on this channel.
uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Deadzone from observed jitter: half the sampled spread, scaled and clamped.
int calcDeadzone(int mn, int mx, int center) {
  int jitter = max(mx - center, center - mn);
  int dz = (int)(jitter * DZ_FACTOR);
  if (dz < DZ_MIN) dz = DZ_MIN;
  if (dz > DZ_MAX) dz = DZ_MAX;
  return dz;
}

// Apply deadzone continuously: 0 inside the band, then ramps from 0 at the edge
// (subtracts the band rather than a hard step, so there's no jump).
int16_t applyDeadzone(int centered, int dz) {
  if (centered >  dz) return centered - dz;
  if (centered < -dz) return centered + dz;
  return 0;
}

// Called after each send with the delivery result.
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("  tx: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("RC starting...");

  pinMode(BTN_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    return;  // ESP-NOW failed to start
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddr, 6);
  peer.channel = 0;      // use current wifi channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("add_peer FAILED");
  }

  // Calibrate: hold the sticks still. Sample the rest position to find each
  // axis's center AND its jitter (min/max spread), then derive the deadzone.
  Serial.println("Calibrating - keep sticks centered & still...");
  const int N = 128;
  long s1x = 0, s1y = 0, s2x = 0;
  int mn1x = 4095, mx1x = 0, mn1y = 4095, mx1y = 0, mn2x = 4095, mx2x = 0;
  for (int i = 0; i < N; i++) {
    int a = analogRead(JOY1_X);
    int b = analogRead(JOY1_Y);
    int c = analogRead(JOY2_X);
    s1x += a; s1y += b; s2x += c;
    if (a < mn1x) mn1x = a;  if (a > mx1x) mx1x = a;
    if (b < mn1y) mn1y = b;  if (b > mx1y) mx1y = b;
    if (c < mn2x) mn2x = c;  if (c > mx2x) mx2x = c;
    delay(4);
  }
  center1X = s1x / N;  center1Y = s1y / N;  center2X = s2x / N;
  dz1X = calcDeadzone(mn1x, mx1x, center1X);
  dz1Y = calcDeadzone(mn1y, mx1y, center1Y);
  dz2X = calcDeadzone(mn2x, mx2x, center2X);
  Serial.print("center X="); Serial.print(center1X);
  Serial.print(" Y=");       Serial.print(center1Y);
  Serial.print(" R=");       Serial.print(center2X);
  Serial.print("   deadzone X="); Serial.print(dz1X);
  Serial.print(" Y=");            Serial.print(dz1Y);
  Serial.print(" R=");            Serial.println(dz2X);
}

void loop() {
  // Debounce the button: only accept a level that's held past the window.
  bool raw = digitalRead(BTN_PIN);
  if (raw != btnLastRead) {
    btnLastRead = raw;
    btnLastChange = millis();
  }
  if (raw != btnStable && (millis() - btnLastChange) >= BTN_DEBOUNCE_MS) {
    btnStable = raw;
    Serial.println(btnStable == LOW ? "  BTN pressed" : "  BTN released");
  }
  bool btnPressed = (btnStable == LOW);  // active-low

  int raw1X = analogRead(JOY1_X);
  int raw1Y = analogRead(JOY1_Y);
  int raw2X = analogRead(JOY2_X);

  JoystickPacket pkt;
  // Center the raw 0..4095 readings around 0, then apply the calibrated
  // per-axis deadzone. If an axis feels inverted, negate it here.
  pkt.x = applyDeadzone(raw1X - center1X, dz1X);   // strafe
  pkt.y = applyDeadzone(raw1Y - center1Y, dz1Y);   // forward/back
  pkt.r = applyDeadzone(raw2X - center2X, dz2X);   // rotation
  pkt.btn = btnPressed ? 1 : 0;

  Serial.print("x=");  Serial.print(pkt.x);
  Serial.print(" y="); Serial.print(pkt.y);
  Serial.print(" r="); Serial.print(pkt.r);
  Serial.print(" btn="); Serial.print(btnPressed ? 1 : 0);

  esp_now_send(broadcastAddr, (uint8_t *)&pkt, sizeof(pkt));

  delay(20);  // ~50 Hz update rate
}
