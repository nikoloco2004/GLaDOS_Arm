/**
 * Servo Channel Probe (PCA9685 + Arduino Uno R4 WiFi)
 *
 * Purpose:
 *   - Help identify which physical servo is on PCA9685 channels 0..3.
 *   - Help discover usable angle limits after rewiring.
 *
 * Behavior:
 *   - On boot, sets all channels to halfway (135 deg) and waits for terminal commands.
 *   - Uses global 0..270 -> tick range mapping, matching your main firmware style.
 *
 * Serial commands (115200):
 *   HELP
 *   CH <0-3> <0-270>                 move one channel to angle
 *   ALL <0-270>                      move all channels to angle
 *   SWEEP <0-3> <min> <max> <step>   sweep one channel
 *   IDENT                            rerun identification sequence
 *   STOP                             stop motion (holds current output)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

constexpr uint8_t PCA_ADDR = 0x40;
constexpr uint8_t SERVO_HZ = 50;
constexpr unsigned long BAUD = 115200;
constexpr unsigned long SERIAL_ATTACH_WAIT_MS = 4000;
constexpr unsigned long HEARTBEAT_MS = 2000;

// Keep this mapping consistent with your main sketch style.
constexpr int ANGLE_MIN = 0;
constexpr int ANGLE_MAX = 270;
constexpr uint16_t PWM_TICK_MIN = 102;
constexpr uint16_t PWM_TICK_MAX = 512;

constexpr uint8_t NUM_CH = 4;
const uint8_t CH[NUM_CH] = {0, 1, 2, 3};

Adafruit_PWMServoDriver pwm(PCA_ADDR);

static bool stopRequested = false;
static unsigned long lastHeartbeatMs = 0;

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

static void setChAngle(uint8_t ch, int deg) {
  if (ch > 15) return;
  pwm.setPWM(ch, 0, angleToTicks(deg));
}

static void moveAll(int deg) {
  for (uint8_t i = 0; i < NUM_CH; ++i) setChAngle(CH[i], deg);
}

static void waitWithStopCheck(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (Serial.available()) return;  // let loop parse command quickly
    delay(2);
  }
}

static void printHelp() {
  Serial.println(F("Servo Channel Probe commands:"));
  Serial.println(F("  HELP"));
  Serial.println(F("  CH <0-3> <0-270>"));
  Serial.println(F("  ALL <0-270>"));
  Serial.println(F("  SWEEP <0-3> <min> <max> <step>"));
  Serial.println(F("  IDENT"));
  Serial.println(F("  PING"));
  Serial.println(F("  STOP"));
}

static void runIdentifySequence() {
  stopRequested = false;
  Serial.println(F("IDENT start"));

  // Neutral-ish starting point.
  moveAll(135);
  waitWithStopCheck(600);

  for (uint8_t i = 0; i < NUM_CH; ++i) {
    if (stopRequested) break;
    uint8_t ch = CH[i];
    Serial.print(F("IDENT channel "));
    Serial.println(ch);

    // Move only this channel so it's obvious which servo responds.
    setChAngle(ch, 0);
    waitWithStopCheck(700);
    if (stopRequested) break;
    setChAngle(ch, 270);
    waitWithStopCheck(700);
    if (stopRequested) break;
    setChAngle(ch, 135);
    waitWithStopCheck(700);
  }

  Serial.println(F("IDENT done"));
}

void setup() {
  Serial.begin(BAUD);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < SERIAL_ATTACH_WAIT_MS)) {
    delay(10);
  }
  delay(200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_HZ);
  delay(10);
  moveAll(135);

  Serial.println(F("OK Servo_Channel_Probe ready"));
  Serial.println(F("All channels set to 135 (halfway)."));
  Serial.println(F("Warning: watch mechanically for end-stops while probing."));
  Serial.print(F("I2C addr: 0x"));
  Serial.println(PCA_ADDR, HEX);
  printHelp();
  Serial.println(F("Waiting for terminal commands..."));
}

void loop() {
  static char buf[96];
  static size_t n = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      buf[n] = '\0';
      n = 0;
      if (buf[0] == '\0') continue;

      char* cmd = strtok(buf, " \t");
      if (!cmd) continue;

      if (strcmp(cmd, "HELP") == 0) {
        printHelp();
      } else if (strcmp(cmd, "PING") == 0) {
        Serial.println(F("PONG"));
      } else if (strcmp(cmd, "STOP") == 0) {
        stopRequested = true;
        Serial.println(F("OK STOP"));
      } else if (strcmp(cmd, "IDENT") == 0) {
        runIdentifySequence();
      } else if (strcmp(cmd, "CH") == 0) {
        char* a = strtok(nullptr, " \t");
        char* b = strtok(nullptr, " \t");
        if (!a || !b) {
          Serial.println(F("ERR CH <0-3> <0-270>"));
          continue;
        }
        int ch = atoi(a);
        int deg = atoi(b);
        if (ch < 0 || ch > 3) {
          Serial.println(F("ERR channel must be 0..3"));
          continue;
        }
        setChAngle((uint8_t)ch, deg);
        Serial.print(F("OK CH "));
        Serial.print(ch);
        Serial.print(' ');
        Serial.println(clampAngle(deg));
      } else if (strcmp(cmd, "ALL") == 0) {
        char* a = strtok(nullptr, " \t");
        if (!a) {
          Serial.println(F("ERR ALL <0-270>"));
          continue;
        }
        int deg = atoi(a);
        moveAll(deg);
        Serial.print(F("OK ALL "));
        Serial.println(clampAngle(deg));
      } else if (strcmp(cmd, "SWEEP") == 0) {
        char* cch = strtok(nullptr, " \t");
        char* cmin = strtok(nullptr, " \t");
        char* cmax = strtok(nullptr, " \t");
        char* cstep = strtok(nullptr, " \t");
        if (!cch || !cmin || !cmax || !cstep) {
          Serial.println(F("ERR SWEEP <0-3> <min> <max> <step>"));
          continue;
        }
        int ch = atoi(cch);
        int amin = clampAngle(atoi(cmin));
        int amax = clampAngle(atoi(cmax));
        int astep = abs(atoi(cstep));
        if (ch < 0 || ch > 3 || astep == 0) {
          Serial.println(F("ERR invalid sweep args"));
          continue;
        }
        stopRequested = false;
        if (amin > amax) {
          int t = amin;
          amin = amax;
          amax = t;
        }
        Serial.print(F("SWEEP channel "));
        Serial.println(ch);
        for (int a = amin; a <= amax; a += astep) {
          if (stopRequested) break;
          setChAngle((uint8_t)ch, a);
          waitWithStopCheck(120);
        }
        for (int a = amax; a >= amin; a -= astep) {
          if (stopRequested) break;
          setChAngle((uint8_t)ch, a);
          waitWithStopCheck(120);
        }
        Serial.println(F("OK SWEEP done"));
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

  unsigned long now = millis();
  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    Serial.println(F("HB ready"));
  }
}
