/**
 * GLaDOS potato arm — Arduino Uno R4 WiFi
 *
 * PCA9685 channels 0..3: wrist, elbow, base, shoulder (same as SET_SERVO order).
 *
 * **Angle → PWM (validated behavior):** each clamped joint angle is converted with a
 * **global** linear map `map(deg, 0, 270, PWM_TICK_MIN, PWM_TICK_MAX)` and passed
 * directly to `setPWM` as the third argument (12-bit tick counts), matching the
 * working prototype. Tune `PWM_TICK_MIN` / `PWM_TICK_MAX` if overall travel needs
 * scaling — not the logical angle limits below (those match Python / config).
 *
 * Libraries: Adafruit PWM Servo Driver Library + Adafruit BusIO
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------------------------------------------------------------------
// PCA9685
// ---------------------------------------------------------------------------
constexpr uint8_t PCA9685_I2C_ADDR = 0x40;
constexpr uint8_t PCA_CHANNEL_WRIST = 0;
constexpr uint8_t PCA_CHANNEL_ELBOW = 1;
constexpr uint8_t PCA_CHANNEL_BASE = 2;
constexpr uint8_t PCA_CHANNEL_SHOULDER = 3;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);

// ================= PCA9685 SETTINGS =================
constexpr uint8_t SERVO_FREQ_HZ = 50;

// PCA9685 "off" tick range for setPWM (third arg) — matched working sketch (102–512)
constexpr uint16_t PWM_TICK_MIN = 102;
constexpr uint16_t PWM_TICK_MAX = 512;

// Global degree span for tick mapping (same for all joints after per-joint clamp)
constexpr int ANGLE_MAP_GLOBAL_MIN = 0;
constexpr int ANGLE_MAP_GLOBAL_MAX = 270;

// ---------------------------------------------------------------------------
// Logical degrees — validated; must match glados_arm/config.py
// ---------------------------------------------------------------------------

// ================= ANGLE LIMITS =================
constexpr int SERVO1_MIN_ANGLE = 0;
constexpr int SERVO1_MAX_ANGLE = 155;

constexpr int SERVO2_MIN_ANGLE = 90;
constexpr int SERVO2_MAX_ANGLE = 270;

constexpr int SERVO3_MIN_ANGLE = 0;
constexpr int SERVO3_MAX_ANGLE = 180;

constexpr int SERVO4_MIN_ANGLE = 0;
constexpr int SERVO4_MAX_ANGLE = 180;

// ================= VALIDATED NEUTRAL POSE =================
constexpr int NEUTRAL_WRIST = 60;
constexpr int NEUTRAL_ELBOW = 270;
constexpr int NEUTRAL_BASE = 90;
constexpr int NEUTRAL_SHOULDER = 0;

// Serial
constexpr unsigned long BAUD_RATE = 115200;
constexpr size_t SERIAL_LINE_MAX = 128;

constexpr unsigned int SERVO_HZ = 50;
constexpr float DEFAULT_SLEW_DEG_PER_SEC = 360.0f;
constexpr unsigned long PCA_RETRY_MS = 500;
constexpr unsigned long PCA_BOOT_INIT_DELAY_MS = 1500;
// Startup hardening: keep first seconds gentle to reduce inrush/current spikes.
constexpr unsigned long PCA_STARTUP_STABILIZE_MS = 2000;
constexpr unsigned long PCA_STARTUP_REFRESH_MS = 800;
constexpr unsigned long PCA_KEEPALIVE_MS = 3000;
constexpr unsigned long PCA_REASSERT_INTERVAL_MS = 80;
constexpr uint8_t PCA_REASSERT_CYCLES = 6;
constexpr unsigned long STARTUP_SERVO_STAGGER_MS = 220;

// ---------------------------------------------------------------------------
enum JointIndex : uint8_t {
  J_WRIST = 0,
  J_ELBOW = 1,
  J_BASE = 2,
  J_SHOULDER = 3,
  NUM_JOINTS = 4
};

constexpr uint8_t kPcaChannel[NUM_JOINTS] = {
  PCA_CHANNEL_WRIST, PCA_CHANNEL_ELBOW, PCA_CHANNEL_BASE, PCA_CHANNEL_SHOULDER};

const int kMinDeg[NUM_JOINTS] = {
  SERVO1_MIN_ANGLE, SERVO2_MIN_ANGLE, SERVO3_MIN_ANGLE, SERVO4_MIN_ANGLE};
const int kMaxDeg[NUM_JOINTS] = {
  SERVO1_MAX_ANGLE, SERVO2_MAX_ANGLE, SERVO3_MAX_ANGLE, SERVO4_MAX_ANGLE};

const int kNeutralDeg[NUM_JOINTS] = {
  NEUTRAL_WRIST, NEUTRAL_ELBOW, NEUTRAL_BASE, NEUTRAL_SHOULDER};

// Safer startup pose (not near end-stops). Boot enters this first; NEUTRAL can be commanded later.
const int kStartupDeg[NUM_JOINTS] = {90, 180, 90, 45};

int currentDeg[NUM_JOINTS];
int targetDeg[NUM_JOINTS];

float slewDegPerSec = DEFAULT_SLEW_DEG_PER_SEC;
bool debugEnabled = false;
unsigned long lastServoMs = 0;
unsigned long lastLoopMs = 0;
unsigned long lastPcaRetryMs = 0;
bool pcaReady = false;
unsigned long bootMs = 0;
unsigned long pcaOnlineSinceMs = 0;
unsigned long lastPcaKeepaliveMs = 0;
unsigned long lastPcaReassertMs = 0;
uint8_t pcaReassertRemaining = 0;

// ---------------------------------------------------------------------------
static int clampJoint(JointIndex j, int v) {
  if (v < kMinDeg[j]) return kMinDeg[j];
  if (v > kMaxDeg[j]) return kMaxDeg[j];
  return v;
}

/// Same rule as working prototype: map(clamped_deg, 0, 270, tick_min, tick_max) → PCA9685 ticks.
static uint16_t angleToPwmTicks(int deg) {
  long t = map(static_cast<long>(deg),
               static_cast<long>(ANGLE_MAP_GLOBAL_MIN),
               static_cast<long>(ANGLE_MAP_GLOBAL_MAX),
               static_cast<long>(PWM_TICK_MIN),
               static_cast<long>(PWM_TICK_MAX));
  if (t < 0L) t = 0L;
  if (t > 4095L) t = 4095L;
  return static_cast<uint16_t>(t);
}

static void setChannelPwmTicks(uint8_t pcaChannel, uint16_t ticks) {
  if (!pcaReady) return;
  if (ticks > 4095U) ticks = 4095U;
  pwm.setPWM(pcaChannel, 0, ticks);
}

static void writeJoint(JointIndex j, int deg) {
  deg = clampJoint(j, deg);
  currentDeg[j] = deg;
  uint16_t ticks = angleToPwmTicks(deg);
  setChannelPwmTicks(kPcaChannel[j], ticks);
}

static void applyTargetsNow() {
  for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
    writeJoint(static_cast<JointIndex>(j), targetDeg[j]);
  }
}

static void applyTargetsStaggered(unsigned long stepDelayMs) {
  for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
    writeJoint(static_cast<JointIndex>(j), targetDeg[j]);
    if (stepDelayMs > 0 && j + 1 < NUM_JOINTS) delay(stepDelayMs);
  }
}

static void applyNeutral() {
  for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
    targetDeg[j] = kNeutralDeg[j];
  }
  applyTargetsNow();
}

static void armPcaReassert() {
  pcaReassertRemaining = PCA_REASSERT_CYCLES;
  lastPcaReassertMs = 0;
}

static void refreshPcaOutputs() {
  if (!pcaReady) return;
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  applyTargetsNow();
}

static void updateSlew() {
  unsigned long now = millis();
  float dt = (lastLoopMs == 0) ? 0.0f : (now - lastLoopMs) / 1000.0f;
  lastLoopMs = now;
  if (dt <= 0.0f || dt > 1.0f) dt = 1.0f / static_cast<float>(SERVO_HZ);

  const float maxStep = slewDegPerSec * dt;

  for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
    int cur = currentDeg[j];
    int tgt = clampJoint(static_cast<JointIndex>(j), targetDeg[j]);
    int diff = tgt - cur;
    if (diff == 0) continue;

    int step;
    if (static_cast<float>(abs(diff)) <= maxStep) {
      step = diff;
    } else {
      step = (diff > 0) ? static_cast<int>(maxStep) : -static_cast<int>(maxStep);
    }
    writeJoint(static_cast<JointIndex>(j), cur + step);
  }
}

static void printStatus() {
  Serial.print(F("OK STATUS driver=PCA9685_ticks wrist="));
  Serial.print(currentDeg[J_WRIST]);
  Serial.print(F(" elbow="));
  Serial.print(currentDeg[J_ELBOW]);
  Serial.print(F(" base="));
  Serial.print(currentDeg[J_BASE]);
  Serial.print(F(" shoulder="));
  Serial.print(currentDeg[J_SHOULDER]);
  Serial.print(F(" slew_dps="));
  Serial.print(slewDegPerSec, 1);
  Serial.print(F(" pca_ready="));
  Serial.print(pcaReady ? F("1") : F("0"));
  Serial.println();
}

static void printHelp() {
  Serial.println(F("GLaDOS_Arm commands (line-based, whitespace-separated):"));
  Serial.println(F("  PING"));
  Serial.println(F("  HELP"));
  Serial.println(F("  NEUTRAL"));
  Serial.println(F("  SET_SERVO <wrist> <elbow> <base> <shoulder>   (integer degrees)"));
  Serial.println(F("  SET_PWM <ch 0-3> <ticks 0-4095>   (bench raw; degree cache unchanged)"));
  Serial.println(F("  SET_SLEW <deg_per_sec>"));
  Serial.println(F("  DEBUG <0|1>"));
  Serial.println(F("  STATUS"));
  Serial.println(F("  I2C_SCAN"));
  Serial.println(F("  PCA_REINIT"));
}

static uint8_t i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

static bool tryInitPca9685() {
  if (i2cProbe(PCA9685_I2C_ADDR) != 0) {
    return false;
  }
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);
  pcaReady = true;
  pcaOnlineSinceMs = millis();
  lastPcaKeepaliveMs = 0;
  armPcaReassert();
  return true;
}

static void printI2cScan() {
  Serial.println(F("I2C scan (0x03-0x77):"));
  uint8_t n = 0;
  for (uint8_t a = 0x03; a < 0x78; ++a) {
    if (i2cProbe(a) == 0) {
      Serial.print(F("  0x"));
      if (a < 16) Serial.print(F("0"));
      Serial.println(a, HEX);
      ++n;
    }
  }
  if (n == 0) Serial.println(F("  (no devices)"));
}

static void handleLine(char* line) {
  char* p = line;
  while (*p == ' ' || *p == '\t') ++p;
  if (*p == '\0') return;

  char* cmd = strtok(p, " \t\r\n");
  if (!cmd) return;

  if (strcmp(cmd, "PING") == 0) {
    Serial.println(F("PONG"));
    return;
  }
  if (strcmp(cmd, "HELP") == 0) {
    printHelp();
    return;
  }
  if (strcmp(cmd, "NEUTRAL") == 0) {
    for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
      targetDeg[j] = kNeutralDeg[j];
    }
    if (pcaReady) {
      applyNeutral();
      Serial.println(F("OK NEUTRAL"));
    } else {
      Serial.println(F("OK NEUTRAL queued (PCA not ready)"));
    }
    if (debugEnabled) printStatus();
    return;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    printStatus();
    return;
  }
  if (strcmp(cmd, "I2C_SCAN") == 0) {
    printI2cScan();
    return;
  }
  if (strcmp(cmd, "PCA_REINIT") == 0) {
    pcaReady = false;
    if (tryInitPca9685()) {
      applyTargetsNow();
      Serial.println(F("OK PCA_REINIT"));
    } else {
      Serial.println(F("ERR PCA_REINIT failed (PCA not detected)"));
    }
    if (debugEnabled) printStatus();
    return;
  }
  if (strcmp(cmd, "DEBUG") == 0) {
    char* a = strtok(nullptr, " \t\r\n");
    if (!a) {
      Serial.println(F("ERR DEBUG needs 0 or 1"));
      return;
    }
    debugEnabled = (atoi(a) != 0);
    Serial.println(debugEnabled ? F("OK DEBUG 1") : F("OK DEBUG 0"));
    return;
  }
  if (strcmp(cmd, "SET_SLEW") == 0) {
    char* a = strtok(nullptr, " \t\r\n");
    if (!a) {
      Serial.println(F("ERR SET_SLEW needs deg_per_sec"));
      return;
    }
    slewDegPerSec = max(0.0f, static_cast<float>(atof(a)));
    Serial.print(F("OK SET_SLEW "));
    Serial.println(slewDegPerSec, 1);
    return;
  }
  if (strcmp(cmd, "SET_PWM") == 0) {
    if (!pcaReady) {
      Serial.println(F("ERR PCA9685 not ready"));
      return;
    }
    char* chs = strtok(nullptr, " \t\r\n");
    char* ts = strtok(nullptr, " \t\r\n");
    if (!chs || !ts) {
      Serial.println(F("ERR SET_PWM <ch 0-3> <ticks 0-4095>"));
      return;
    }
    int ch = atoi(chs);
    int tk = atoi(ts);
    if (ch < 0 || ch > 3) {
      Serial.println(F("ERR channel must be 0-3"));
      return;
    }
    if (tk < 0 || tk > 4095) {
      Serial.println(F("ERR ticks must be 0-4095"));
      return;
    }
    setChannelPwmTicks(static_cast<uint8_t>(ch), static_cast<uint16_t>(tk));
    Serial.println(F("OK SET_PWM (degree cache unchanged; use SET_SERVO to resync)"));
    return;
  }
  if (strcmp(cmd, "SET_SERVO") == 0) {
    char* ws = strtok(nullptr, " \t\r\n");
    char* es = strtok(nullptr, " \t\r\n");
    char* bs = strtok(nullptr, " \t\r\n");
    char* ss = strtok(nullptr, " \t\r\n");
    if (!ws || !es || !bs || !ss) {
      Serial.println(F("ERR SET_SERVO needs 4 integer values"));
      return;
    }
    int w = atoi(ws);
    int e = atoi(es);
    int b = atoi(bs);
    int s = atoi(ss);

    targetDeg[J_WRIST] = clampJoint(J_WRIST, w);
    targetDeg[J_ELBOW] = clampJoint(J_ELBOW, e);
    targetDeg[J_BASE] = clampJoint(J_BASE, b);
    targetDeg[J_SHOULDER] = clampJoint(J_SHOULDER, s);

    if (!pcaReady) {
      Serial.println(F("OK SET_SERVO queued (PCA not ready)"));
    } else {
      if (slewDegPerSec <= 0.0f) {
        for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
          writeJoint(static_cast<JointIndex>(j), targetDeg[j]);
        }
        Serial.println(F("OK SET_SERVO"));
      } else {
        Serial.println(F("OK SET_SERVO (slewing)"));
      }
    }
    if (debugEnabled) printStatus();
    return;
  }

  Serial.print(F("ERR UNKNOWN "));
  Serial.println(cmd);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_RATE);
  delay(200);
  bootMs = millis();

  Wire.begin();

  // Start with safer (non-end-stop) boot targets even if hardware is not yet online.
  for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
    targetDeg[j] = kStartupDeg[j];
    currentDeg[j] = kStartupDeg[j];
  }

  Serial.println(F("INFO delaying initial PCA init for PSU stabilization"));
  Serial.print(F("INFO PCA init delay ms="));
  Serial.println(PCA_BOOT_INIT_DELAY_MS);

  Serial.println(F("OK GLaDOS_Arm ready (PCA9685 tick map 0-270 -> PWM_TICK_MIN/MAX)"));
  printHelp();
}

void loop() {
  static char lineBuf[SERIAL_LINE_MAX];
  static size_t lineLen = 0;

  unsigned long now = millis();
  if (!pcaReady) {
    if (now - bootMs >= PCA_BOOT_INIT_DELAY_MS && (now - lastPcaRetryMs >= PCA_RETRY_MS)) {
      lastPcaRetryMs = now;
      if (tryInitPca9685()) {
        applyTargetsStaggered(STARTUP_SERVO_STAGGER_MS);
        Serial.println(F("OK PCA9685 online; safe startup pose applied"));
        Serial.println(F("INFO use NEUTRAL command after boot if you want calibrated neutral pose"));
        if (debugEnabled) printStatus();
      }
    }
  } else {
    if (i2cProbe(PCA9685_I2C_ADDR) != 0) {
      pcaReady = false;
      Serial.println(F("WARN PCA9685 dropped off I2C; retrying init"));
    } else {
      const bool startupWindow = (now - pcaOnlineSinceMs) < PCA_STARTUP_STABILIZE_MS;
      const unsigned long refreshMs = startupWindow ? PCA_STARTUP_REFRESH_MS : PCA_KEEPALIVE_MS;
      if (lastPcaKeepaliveMs == 0 || now - lastPcaKeepaliveMs >= refreshMs) {
        lastPcaKeepaliveMs = now;
        refreshPcaOutputs();
      }
    }
  }

  if (pcaReady && pcaReassertRemaining > 0 &&
      (lastPcaReassertMs == 0 || now - lastPcaReassertMs >= PCA_REASSERT_INTERVAL_MS)) {
    lastPcaReassertMs = now;
    applyTargetsNow();
    --pcaReassertRemaining;
  }

  if (now - lastServoMs >= 1000UL / SERVO_HZ) {
    lastServoMs = now;
    if (pcaReady && slewDegPerSec > 0.0f) {
      updateSlew();
    }
  }

  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      lineLen = 0;
      if (lineBuf[0] != '\0') handleLine(lineBuf);
      continue;
    }
    if (lineLen < SERIAL_LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
      Serial.println(F("ERR LINE_TOO_LONG"));
    }
  }
}
