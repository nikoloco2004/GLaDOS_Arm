/**
 * R4 Direct Servo 90 Test (NO I2C / NO PCA9685)
 *
 * Drives one servo directly from an Arduino Uno R4 WiFi digital pin.
 *
 * Wiring:
 *   - Servo signal -> SERVO_PIN (default D9)
 *   - Servo V+ from external 5-6V supply
 *   - Servo GND tied to external supply GND and Arduino GND
 *
 * Serial (115200):
 *   - "PING"         -> PONG
 *   - "ANGLE <0-180>" -> set direct servo angle
 */

#include <Arduino.h>
#include <Servo.h>

constexpr unsigned long BAUD = 115200;
constexpr uint8_t SERVO_PIN = 9;   // change if needed
constexpr int BOOT_ANGLE = 135;     // requested test angle

Servo testServo;

static int clampAngle(int a) {
  if (a < 0) return 0;
  if (a > 180) return 0;
  return a;
}

void setup() {
  Serial.begin(BAUD);
  delay(200);

  testServo.attach(SERVO_PIN);
  testServo.write(BOOT_ANGLE);

  Serial.println(F("OK R4_Direct_Servo_90_Test ready"));
  Serial.print(F("Pin D"));
  Serial.print(SERVO_PIN);
  Serial.print(F(" -> "));
  Serial.print(BOOT_ANGLE);
  Serial.println(F(" deg"));
  Serial.println(F("Commands: PING | ANGLE <0-180>"));
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
      } else if (strcmp(cmd, "ANGLE") == 0) {
        char* a = strtok(nullptr, " \t");
        if (!a) {
          Serial.println(F("ERR ANGLE <0-180>"));
          continue;
        }
        int angle = clampAngle(atoi(a));
        testServo.write(angle);
        Serial.print(F("OK ANGLE "));
        Serial.println(angle);
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

