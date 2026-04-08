/**
 * R4 Servo 90 Test (Arduino Uno R4 WiFi + PCA9685)
 *
 * Purpose:
 *   Minimal standalone test to drive one PCA9685 channel to 90 degrees.
 *
 * Wiring:
 *   - Uno R4 WiFi SDA/SCL -> PCA9685 SDA/SCL
 *   - Shared GND
 *   - Servo powered from PCA9685 V+ rail (not USB 5V)
 *
 * Serial (115200):
 *   - "CH <0-15>" : move that channel to 90 deg
 *   - "PING"      : replies "PONG"
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

constexpr uint8_t PCA_ADDR = 0x40;
constexpr uint8_t SERVO_HZ = 50;
constexpr unsigned long BAUD = 115200;

// Same mapping style used in your main firmware.
constexpr int ANGLE_MIN = 0;
constexpr int ANGLE_MAX = 270;
constexpr uint16_t PWM_TICK_MIN = 102;
constexpr uint16_t PWM_TICK_MAX = 512;

constexpr uint8_t DEFAULT_CHANNEL = 2;  // your current shoulder channel mapping
constexpr int TARGET_DEG = 90;

Adafruit_PWMServoDriver pwm(PCA_ADDR);

static int clampAngle(int a) {
  if (a < ANGLE_MIN) return ANGLE_MIN;
  if (a > ANGLE_MAX) return ANGLE_MAX;
  return a;
}

static uint16_t angleToTicks(int deg) {
  deg = clampAngle(deg);
  long t = map((long)deg, (long)ANGLE_MIN, (long)ANGLE_MAX, (long)PWM_TICK_MIN, (long)PWM_TICK_MAX);
  if (t < 0L) t = 0L;
  if (t > 4095L) t = 4095L;
  return (uint16_t)t;
}

static void setChannelDeg(uint8_t ch, int deg) {
  if (ch > 15) return;
  pwm.setPWM(ch, 0, angleToTicks(deg));
}

void setup() {
  Serial.begin(BAUD);
  delay(200);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_HZ);
  delay(20);

  setChannelDeg(DEFAULT_CHANNEL, TARGET_DEG);

  Serial.println(F("OK R4_Servo_90_Test ready"));
  Serial.print(F("Set CH "));
  Serial.print(DEFAULT_CHANNEL);
  Serial.print(F(" -> "));
  Serial.print(TARGET_DEG);
  Serial.println(F(" deg"));
  Serial.println(F("Commands: PING | CH <0-15>"));
}

void loop() {
  static char buf[64];
  static size_t n = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[n] = '\0';
      n = 0;
      if (buf[0] == '\0') continue;

      char* cmd = strtok(buf, " \t");
      if (!cmd) continue;

      if (strcmp(cmd, "PING") == 0) {
        Serial.println(F("PONG"));
      } else if (strcmp(cmd, "CH") == 0) {
        char* a = strtok(nullptr, " \t");
        if (!a) {
          Serial.println(F("ERR CH <0-15>"));
          continue;
        }
        int ch = atoi(a);
        if (ch < 0 || ch > 15) {
          Serial.println(F("ERR channel must be 0..15"));
          continue;
        }
        setChannelDeg((uint8_t)ch, TARGET_DEG);
        Serial.print(F("OK CH "));
        Serial.print(ch);
        Serial.print(F(" -> "));
        Serial.print(TARGET_DEG);
        Serial.println(F(" deg"));
      } else {
        Serial.print(F("ERR UNKNOWN "));
        Serial.println(cmd);
      }
      continue;
    }

    if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;
      Serial.println(F("ERR line too long"));
    }
  }
}

