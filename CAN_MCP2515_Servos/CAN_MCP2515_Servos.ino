#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─── CONFIG ──────────────────────────────────────────────────────────────────
// MCP2515 (CAN) pins on Arduino Uno:
static const uint8_t MCP_CS   = 10;   
static const uint8_t MCP_INT  = 2;    

// PCA9685 (I²C) default address 0x40
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// CAN bus:
MCP2515 can(MCP_CS);
static const uint32_t CAN_BAUD = CAN_500KBPS;
static const uint16_t CMD_SERVO = 0x200;  // COB‑ID for Set‑Servo

// Servo pulse bounds (out of 4096) for SG90 on PCA9685 @50 Hz
static const uint16_t US_MIN = 150;  // ~1.0 ms
static const uint16_t US_MAX = 600;  // ~2.0 ms

// ─── SETUP ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while(!Serial);

  // 1) Init I²C/PCA9685
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);  // servos run at 50 Hz

  // 2) Init SPI/CAN
  SPI.begin();
  can.reset();
  if (can.setBitrate(CAN_BAUD, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println("❌ CAN setBitrate failed");
    while(1);
  }
  if (can.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("❌ CAN setNormalMode failed");
    while(1);
  }

  Serial.println("✓ PCA9685 @50Hz and CAN @500kbps ready");
}

// map degrees (0–180) → pulse length (US_MIN–US_MAX)
uint16_t angleToTicks(uint8_t angle) {
  if (angle > 180) angle = 180;
  return US_MIN + (uint32_t)(US_MAX - US_MIN) * angle / 180;
}

// ─── LOOP ───────────────────────────────────────────────────────────────────
void loop() {
  struct can_frame frame;
  // poll for a message
  if (can.readMessage(&frame) == MCP2515::ERROR_OK) {
    if (frame.can_id == CMD_SERVO && frame.can_dlc >= 2) {
      uint8_t channel = frame.data[0];      // 0–15
      uint8_t angle   = frame.data[1];      // 0–180
      if (channel < 16) {
        uint16_t ticks = angleToTicks(angle);
        pca.setPWM(channel, 0, ticks);
        Serial.print("→ Servo "); Serial.print(channel);
        Serial.print(" → "); Serial.print(angle); Serial.print("°  (");
        Serial.print(ticks); Serial.println(" ticks)");
      }
    }
  }
}
