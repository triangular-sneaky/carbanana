// ESP32  --  RC (transmitter)
// Reads a joystick (X on VP/GPIO36, Y on VN/GPIO39) and sends the values to the
// car over ESP-NOW using the broadcast address (no MAC to hardcode).

#include <esp_now.h>
#include <WiFi.h>

// ---- Joystick pins ---------------------------------------------------------
const int JOY_X = 36;  // VP  (SENSOR_VP) - ADC1_CH0
const int JOY_Y = 39;  // VN  (SENSOR_VN) - ADC1_CH3

// Joystick center is measured at boot (this stick rests near ~1870, not 2048).
// Keep the stick centered during power-up / reset.
int centerX = 2048;
int centerY = 2048;

// ---- Wireless packet (must match car side) ---------------------------------
typedef struct {
  int16_t x;  // steering, centered at 0  (roughly -2048 .. +2048)
  int16_t y;  // throttle, centered at 0  (roughly -2048 .. +2048)
} JoystickPacket;

// Broadcast to any ESP-NOW peer listening on this channel.
uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Called after each send with the delivery result.
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("  tx: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("RC starting...");

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

  // Calibrate: average the resting stick to find true center.
  long sumX = 0, sumY = 0;
  const int N = 64;
  for (int i = 0; i < N; i++) {
    sumX += analogRead(JOY_X);
    sumY += analogRead(JOY_Y);
    delay(5);
  }
  centerX = sumX / N;
  centerY = sumY / N;
  Serial.print("center X="); Serial.print(centerX);
  Serial.print(" Y=");       Serial.println(centerY);
}

void loop() {
  int rawX = analogRead(JOY_X);
  int rawY = analogRead(JOY_Y);

  JoystickPacket pkt;
  // Center the raw 0..4095 readings around 0.
  // If forward/back or turning feels inverted, negate the axis here.
  pkt.x = rawX - centerX;
  pkt.y = rawY - centerY;

  Serial.print("rawX=");  Serial.print(rawX);
  Serial.print(" rawY="); Serial.print(rawY);
  Serial.print("  ->  x="); Serial.print(pkt.x);
  Serial.print(" y=");      Serial.print(pkt.y);

  esp_now_send(broadcastAddr, (uint8_t *)&pkt, sizeof(pkt));

  delay(200);  // slow down for readable serial output while debugging
}
