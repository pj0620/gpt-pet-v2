/*
  ## Motor Diagram:
  ^
  | Forward

  L1 +------+ R1
     |      |
     |      |
     |      |
  L2 +------+ R2

  Motors: JGB37-520 12V 76 rpm gear motors with A/B Hall encoders.
  All speeds are output-shaft rpm; +rpm is forward on every motor.

  ## Serial protocol (115200 8N1, little-endian)
  The first byte of each command selects its format:
  - 'W' (host, binary): 'W' + 4 x int16 (rpm * 100), order L1, L2, R1, R2,
      + 1 byte XOR checksum of the 8 payload bytes. 10 bytes total. Signed, so negative is
      reverse. Targets beyond +-max_rpm are clamped. A bad checksum drops the packet.
      This is what the ROS2 hardware interface (ros2_ws/src/bot_ws/src/gptpet_hardware) sends,
      50 times a second.
  - 'V' (legacy, binary): 'V' + 4 bytes, bit 7 = reverse, bits 0..6 = 0..127 -> 0..max_rpm.
  - anything else: a newline-terminated text command (Serial Monitor / Plotter input box, and
    the hardware interface for parameter changes):
      S <motor> <rpm>           motor 0..3 = L1, L2, R1, R2; rpm is signed, e.g. S 0 40 or S 3 -34
      set_speed <motor> <rpm>   long form of S
      stop                      all motors off
      set <name> <value>        change a parameter, see Parameters below
      get <name>                print one parameter as name=value
      params                    print every parameter on one line
      plot on | plot off        Serial Plotter mode, see Output below. Default off.
      status                    one-line summary of mode, watchdog, targets and measurements
    Replies are single words (OK_..., ERR_...) so the Serial Plotter doesn't graph them.

  ## Parameters
  Everything tunable can be changed at runtime with 'set <name> <value>'. Per-motor names take
  an optional motor index 0..3; without it the value applies to all four motors.
      kp ki kd        PID gains, PWM counts per rpm (kp), per rpm*s (ki), per rpm/s (kd)
      enc_sign        1 or -1: flip if a motor's measured speed has the opposite sign to its target
      reversed        0 or 1: motor wired backwards (PWM and measured sign are both flipped)
      floor           PID output floor in PWM counts, 0..254. Also the slowest the robot can go.
      max_rpm         target clamp, 1..500. Also the full-scale value of the legacy 'V' command.
      watchdog_ms     stop the motors after this long without a 'W'/'V' command, 50..10000
      sample_ms       speed measurement and PID period, 5..200
      report_ms       binary report period, 5..1000
      open_loop       0 or 1: bypass the PID and map the target straight to PWM
  Examples: set kp 3.5 / set kp2 4 / set floor 30 / get kp / params
  Values are not saved across resets: the defaults below apply at boot, and ROS pushes its
  parameter set (motor_tuning.yaml, /gptpet_motors) every time the hardware interface
  activates, so ROS is the source of truth while the robot is running.

  ## Watchdog
  After a 'W' or 'V' command, if no further one arrives within watchdog_ms all motors are
  stopped (prints WATCHDOG_stopped once). Text commands are not covered by it, so manual
  tuning from the Serial Monitor keeps running.

  ## Control modes (open_loop parameter)
  - open_loop = 0: closed loop. Each command is a target speed and a PID per motor drives
    the PWM using encoder feedback.
  - open_loop = 1: open loop (dead reckoning). Each command is mapped straight to PWM
    (0..max_rpm -> 0..255) and written to the motor; the PID is bypassed. Encoders are
    still read, but they don't affect the output.

  ## Output
  - Host mode (default): every report_ms a binary report, 27 bytes:
      'M', 24, then per motor (L1, L2, R1, R2): int16 measured rpm * 100 (signed, taken from
      the encoder so it is the real direction) + int32 encoder counts (+ is forward), then
      1 byte XOR checksum of the 24 payload bytes.
  - Plotter mode ('plot on'): every PLOT_INTERVAL_MS, target vs measured rpm per motor for the
    Arduino Serial Plotter, and no binary reports:
      target0:40.0,measured0:39.6,target1:0.0,measured1:0.0,...
*/

// #include <WiFi.h>
// #include <ESPmDNS.h>
// #include <ESPAsyncWebServer.h>
// #include <Preferences.h>
#include <PID_v2.h>

// ---- Defaults. All of these can be changed at runtime with 'set <name> <value>'. ----
#define MAX_RPM 76.0             // rated no-load speed at 12V; targets are clamped here
// PID output floor, so the motor doesn't sit in its deadband. This is also the slowest the
// robot can go: at 75 the wheels ran 14-20 rpm (0.07-0.10 m/s) whatever the target, which is
// already Nav2's top speed. Tune live with 'set floor <pwm>' and copy the value here.
#define MIN_PWM 35
#define SPEED_SAMPLE_MS 20       // speed measurement and PID period
#define REPORT_INTERVAL_MS 20    // binary 'M' report period (host mode)
#define HOST_CMD_TIMEOUT_MS 500  // stop the motors if the host goes quiet for this long

#define PLOT_INTERVAL_MS 50      // Serial Plotter line period (plotter mode)
#define SERIAL_TIMEOUT_MS 20     // how long to wait for the rest of a command

// Encoder: 11 pulses per motor revolution per channel, decoded x4 (every edge of A and B),
// through a 131:1 gearbox (the 76 rpm version). If measured speed at full PWM with no load
// is far from ~76 rpm, the gear ratio is different -- adjust GEAR_RATIO.
// The host converts counts to wheel angle with the same number (encoder_counts_per_rev).
#define ENC_PPR 11
#define GEAR_RATIO 131.0
#define COUNTS_PER_REV (ENC_PPR * 4 * GEAR_RATIO)

// Serial framing, shared with the host (ros2_ws/src/bot_ws/src/gptpet_hardware)
#define RPM_SCALE 100.0          // rpm travel over the wire as int16 rpm * RPM_SCALE
#define CMD_HEADER 'W'
#define CMD_PAYLOAD_LEN 8        // 4 x int16
#define LEGACY_CMD_HEADER 'V'
#define REPORT_HEADER 'M'
#define REPORT_PAYLOAD_LEN 24    // 4 x (int16 rpm + int32 counts)

// false -> host mode: binary 'M' reports, no plot lines. true -> Serial Plotter text.
// Toggle at runtime with 'plot on' / 'plot off'.
bool plotMode = false;

// ---- Runtime parameters (see Parameters in the header) ----
// Starting gains for rpm units; tune per motor if needed
double kps[4] = { 3,  3,  3,  3  };
double kis[4] = { 15, 15, 15, 15 };
double kds[4] = { 0,  0,  0,  0  };

// Motors wired backwards (a positive command drove them in reverse). For these, the PWM sent
// to the driver and the measured speed sign are both flipped, so +rpm means forward on every
// motor and commands, targets and reported speeds all stay in that frame.
//                        L1(0)  L2(1)  R1(2)  R2(3)
bool motorReversed[4] = { true,  true,  false, true };

// Set to -1 for any motor whose measured speed has the opposite sign to its target
// (its encoder A/B wires are the other way round). Fix the sign here, not in motorReversed.
// Only affects the reported sign, not control.
int encSign[4] = { 1, 1, 1, 1 };

int minPwm = MIN_PWM;
double maxRpm = MAX_RPM;
unsigned long watchdogMs = HOST_CMD_TIMEOUT_MS;
int sampleMs = SPEED_SAMPLE_MS;
int reportMs = REPORT_INTERVAL_MS;
bool openLoop = false;

// L1+, L1-, L2+, L2-, R1+, R1-, R2+, R2-
const int motorPins[] = {15, 16, 17, 18, 4, 5, 6, 7};

// Encoder pins per motor {A, B}: L1, L2, R1, R2
int encPins[4][2] = {
  {35, 37},
  {41, 38},
  {40, 42},
  {36, 39},
};

// Quadrature decoder: index = (previous AB state << 2) | new AB state
DRAM_ATTR static const int8_t QDEC_TABLE[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

// Running encoder counts, written only by the ISRs
volatile int32_t encCounts[4] = { 0, 0, 0, 0 };
volatile uint8_t encStates[4] = { 0, 0, 0, 0 };
int32_t lastEncCounts[4] = { 0, 0, 0, 0 };
unsigned long lastSpeedSampleUs = 0;

// Speed magnitudes in rpm, used by the PIDs: v_L1, v_L2, v_R1, v_R2
static double measuredSpeeds[4] = {
  0, 0, 0, 0
};
static double desiredSpeeds[4] = {
  0, 0, 0, 0
};

// Signed measured rpm, for reporting
static double signedRpms[4] = {
  0, 0, 0, 0
};

// 1 -> forward, -1 -> reverse
double directions[4] = {
  1, 1, 1, 1
};

static double controlSpeeds[4] = {
  0, 0, 0, 0
};

// open-loop PWM per motor in [-255, 255], only used when openLoop is true
static double directPwm[4] = {
  0, 0, 0, 0
};

PID l1Pid(&measuredSpeeds[0], &controlSpeeds[0], &desiredSpeeds[0], kps[0], kis[0], kds[0], DIRECT);
PID l2Pid(&measuredSpeeds[1], &controlSpeeds[1], &desiredSpeeds[1], kps[1], kis[1], kds[1], DIRECT);
PID r1Pid(&measuredSpeeds[2], &controlSpeeds[2], &desiredSpeeds[2], kps[2], kis[2], kds[2], DIRECT);
PID r2Pid(&measuredSpeeds[3], &controlSpeeds[3], &desiredSpeeds[3], kps[3], kis[3], kds[3], DIRECT);
static PID* pidControllers[4] = { &l1Pid, &l2Pid, &r1Pid, &r2Pid};

unsigned long lastReportTime = 0;
unsigned long lastPlotTime = 0;

// Host watchdog: armed by a 'W'/'V' command, disarmed by text commands or when it fires
bool hostCmdArmed = false;
unsigned long lastHostCmdMs = 0;
unsigned long badChecksums = 0;

// Fires on every edge of either encoder channel of motor i
void IRAM_ATTR onEncoderEdge(void* arg) {
  int i = (int)(intptr_t)arg;
  uint8_t state = (digitalRead(encPins[i][0]) << 1) | digitalRead(encPins[i][1]);
  encCounts[i] = encCounts[i] + QDEC_TABLE[(encStates[i] << 2) | state];
  encStates[i] = state;
}

// Converts encoder counts since the last sample into rpm, every sampleMs
void updateSpeedMeasurements() {
  unsigned long nowUs = micros();
  unsigned long elapsedUs = nowUs - lastSpeedSampleUs;
  if (elapsedUs < (unsigned long)sampleMs * 1000UL) {
    return;
  }
  lastSpeedSampleUs = nowUs;

  for (int i = 0; i < 4; i++) {
    int32_t count = encCounts[i];
    int32_t delta = count - lastEncCounts[i];
    lastEncCounts[i] = count;

    double rpm = encSign[i] * (delta / COUNTS_PER_REV) * (60e6 / elapsedUs);
    signedRpms[i] = motorReversed[i] ? -rpm : rpm;
    measuredSpeeds[i] = fabs(signedRpms[i]);
  }
}

// Encoder counts of motor i in the command frame (+ is forward), same frame as signedRpms
int32_t forwardCounts(int i) {
  int32_t raw = encCounts[i];
  int32_t c = raw * encSign[i];
  return motorReversed[i] ? -c : c;
}

void writePair(int positivePin, int negativePin, double value) {
  if (value > 0) {
    analogWrite(positivePin, value);
    analogWrite(negativePin, 0);
  }
  else {
    analogWrite(positivePin, 0);
    analogWrite(negativePin, -1.0 * value);
  }
}

// Drives motor i with a signed PWM in [-255, 255], flipping it for motors wired backwards
void writeMotor(int i, double value) {
  if (motorReversed[i]) {
    value = -value;
  }
  writePair(motorPins[2 * i], motorPins[2 * i + 1], value);
}

// Sets one motor's target from a signed rpm, clamped to +-maxRpm
void setMotorTarget(int i, double rpm) {
  if (rpm > maxRpm) rpm = maxRpm;
  if (rpm < -maxRpm) rpm = -maxRpm;

  directions[i] = (rpm < 0) ? -1.0 : 1.0;
  desiredSpeeds[i] = fabs(rpm);
  directPwm[i] = rpm / maxRpm * 255.0;
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    setMotorTarget(i, 0);
  }
}

bool anyTargetSet() {
  for (int i = 0; i < 4; i++) {
    if (desiredSpeeds[i] > 0.001) return true;
  }
  return false;
}

void armWatchdog() {
  lastHostCmdMs = millis();
  hostCmdArmed = true;
}

// Stops the motors when the host stops sending commands (crash, USB unplugged, ...)
void checkWatchdog() {
  if (!hostCmdArmed) {
    return;
  }
  if (millis() - lastHostCmdMs < watchdogMs) {
    return;
  }
  hostCmdArmed = false;
  bool wasMoving = anyTargetSet();
  stopAllMotors();
  if (wasMoving) {
    Serial.println("WATCHDOG_stopped");
  }
}

// ---------------------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------------------

// Matches "<base>" (idx = -1, all motors) or "<base><0..3>" (idx = that motor)
bool matchMotorParam(const char* name, const char* base, int* idx) {
  size_t n = strlen(base);
  if (strncmp(name, base, n) != 0) {
    return false;
  }
  if (name[n] == '\0') {
    *idx = -1;
    return true;
  }
  if (name[n] >= '0' && name[n] <= '3' && name[n + 1] == '\0') {
    *idx = name[n] - '0';
    return true;
  }
  return false;
}

void applyPidTunings(int i) {
  pidControllers[i]->SetTunings(kps[i], kis[i], kds[i]);
}

// Sets one parameter. Returns NULL on success, otherwise an error word to print.
const char* setParam(const char* name, double value) {
  int idx;

  if (matchMotorParam(name, "kp", &idx) || matchMotorParam(name, "ki", &idx) ||
      matchMotorParam(name, "kd", &idx)) {
    if (value < 0) {
      return "ERR_gain_must_be_positive";
    }
    double* gains = (name[1] == 'p') ? kps : (name[1] == 'i') ? kis : kds;
    for (int i = 0; i < 4; i++) {
      if (idx == -1 || idx == i) {
        gains[i] = value;
        applyPidTunings(i);
      }
    }
    return NULL;
  }
  if (matchMotorParam(name, "enc_sign", &idx)) {
    if (value != 1 && value != -1) {
      return "ERR_enc_sign_must_be_1_or_-1";
    }
    for (int i = 0; i < 4; i++) {
      if (idx == -1 || idx == i) encSign[i] = (int)value;
    }
    return NULL;
  }
  if (matchMotorParam(name, "reversed", &idx)) {
    if (value != 0 && value != 1) {
      return "ERR_reversed_must_be_0_or_1";
    }
    for (int i = 0; i < 4; i++) {
      if (idx == -1 || idx == i) motorReversed[i] = (value != 0);
    }
    return NULL;
  }
  if (strcmp(name, "floor") == 0) {
    if (value < 0 || value > 254) {
      return "ERR_floor_must_be_0_to_254";
    }
    minPwm = (int)value;
    for (int i = 0; i < 4; i++) {
      pidControllers[i]->SetOutputLimits(minPwm, 255);
    }
    return NULL;
  }
  if (strcmp(name, "max_rpm") == 0) {
    if (value < 1 || value > 500) {
      return "ERR_max_rpm_must_be_1_to_500";
    }
    maxRpm = value;
    for (int i = 0; i < 4; i++) {
      setMotorTarget(i, directions[i] * desiredSpeeds[i]);  // re-clamp current targets
    }
    return NULL;
  }
  if (strcmp(name, "watchdog_ms") == 0) {
    if (value < 50 || value > 10000) {
      return "ERR_watchdog_ms_must_be_50_to_10000";
    }
    watchdogMs = (unsigned long)value;
    return NULL;
  }
  if (strcmp(name, "sample_ms") == 0) {
    if (value < 5 || value > 200) {
      return "ERR_sample_ms_must_be_5_to_200";
    }
    sampleMs = (int)value;
    for (int i = 0; i < 4; i++) {
      pidControllers[i]->SetSampleTime(sampleMs);
    }
    return NULL;
  }
  if (strcmp(name, "report_ms") == 0) {
    if (value < 5 || value > 1000) {
      return "ERR_report_ms_must_be_5_to_1000";
    }
    reportMs = (int)value;
    return NULL;
  }
  if (strcmp(name, "open_loop") == 0) {
    if (value != 0 && value != 1) {
      return "ERR_open_loop_must_be_0_or_1";
    }
    openLoop = (value != 0);
    return NULL;
  }
  return "ERR_unknown_parameter";
}

void printDoubleArray(const double* values) {
  for (int i = 0; i < 4; i++) {
    Serial.printf("%s%g", i > 0 ? "," : "", values[i]);
  }
}

void printIntArray(const int* values) {
  for (int i = 0; i < 4; i++) {
    Serial.printf("%s%d", i > 0 ? "," : "", values[i]);
  }
}

void printBoolArray(const bool* values) {
  for (int i = 0; i < 4; i++) {
    Serial.printf("%s%d", i > 0 ? "," : "", values[i] ? 1 : 0);
  }
}

// Prints "<name>=<value>" (or "<name>=v0,v1,v2,v3" for a per-motor name without an index).
// Returns false if the name is unknown.
bool printParam(const char* name) {
  int idx;
  if (matchMotorParam(name, "kp", &idx) || matchMotorParam(name, "ki", &idx) ||
      matchMotorParam(name, "kd", &idx)) {
    const double* gains = (name[1] == 'p') ? kps : (name[1] == 'i') ? kis : kds;
    Serial.printf("%s=", name);
    if (idx == -1) printDoubleArray(gains); else Serial.printf("%g", gains[idx]);
    Serial.println();
    return true;
  }
  if (matchMotorParam(name, "enc_sign", &idx)) {
    Serial.printf("%s=", name);
    if (idx == -1) printIntArray(encSign); else Serial.printf("%d", encSign[idx]);
    Serial.println();
    return true;
  }
  if (matchMotorParam(name, "reversed", &idx)) {
    Serial.printf("%s=", name);
    if (idx == -1) printBoolArray(motorReversed); else Serial.printf("%d", motorReversed[idx] ? 1 : 0);
    Serial.println();
    return true;
  }
  if (strcmp(name, "floor") == 0)       { Serial.printf("floor=%d\n", minPwm); return true; }
  if (strcmp(name, "max_rpm") == 0)     { Serial.printf("max_rpm=%g\n", maxRpm); return true; }
  if (strcmp(name, "watchdog_ms") == 0) { Serial.printf("watchdog_ms=%lu\n", watchdogMs); return true; }
  if (strcmp(name, "sample_ms") == 0)   { Serial.printf("sample_ms=%d\n", sampleMs); return true; }
  if (strcmp(name, "report_ms") == 0)   { Serial.printf("report_ms=%d\n", reportMs); return true; }
  if (strcmp(name, "open_loop") == 0)   { Serial.printf("open_loop=%d\n", openLoop ? 1 : 0); return true; }
  return false;
}

// Every parameter on one line, e.g.
//   PARAMS kp=3,3,3,3 ki=15,15,15,15 kd=0,0,0,0 enc_sign=1,1,1,1 reversed=1,1,0,1 floor=35 ...
void printParams() {
  Serial.print("PARAMS kp=");
  printDoubleArray(kps);
  Serial.print(" ki=");
  printDoubleArray(kis);
  Serial.print(" kd=");
  printDoubleArray(kds);
  Serial.print(" enc_sign=");
  printIntArray(encSign);
  Serial.print(" reversed=");
  printBoolArray(motorReversed);
  Serial.printf(" floor=%d max_rpm=%g watchdog_ms=%lu sample_ms=%d report_ms=%d open_loop=%d\n",
                minPwm, maxRpm, watchdogMs, sampleMs, reportMs, openLoop ? 1 : 0);
}

// ---------------------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------------------

/*
  Host command:
    'W' + 4 x int16 LE (rpm * RPM_SCALE), order L1, L2, R1, R2, + XOR checksum of the 8 bytes
  The header byte has already been consumed.
*/
void handleRpmCommand() {
  uint8_t buf[CMD_PAYLOAD_LEN + 1];
  if (Serial.readBytes(buf, sizeof(buf)) != sizeof(buf)) {
    return;  // incomplete packet: ignore it, the host repeats at 50 Hz
  }

  uint8_t cs = 0;
  for (int k = 0; k < CMD_PAYLOAD_LEN; k++) {
    cs ^= buf[k];
  }
  if (cs != buf[CMD_PAYLOAD_LEN]) {
    badChecksums++;
    return;
  }

  for (int i = 0; i < 4; i++) {
    int16_t scaled = (int16_t)((uint16_t)buf[2 * i] | ((uint16_t)buf[2 * i + 1] << 8));
    setMotorTarget(i, scaled / RPM_SCALE);
  }
  armWatchdog();
}

/*
  Legacy host command:
    'V' V_L1 V_L2 V_R1 V_R2   (one byte each)
  Where bit 7 is the direction and bits 0..6 are the speed 0..127 (0..maxRpm)
*/
void handleVelocitiesCommand() {
  uint8_t bytes[4];
  if (Serial.readBytes(bytes, 4) != 4) {
    return;  // a missing byte used to be read as -1, i.e. full speed reverse
  }

  for (int i = 0; i < 4; i++) {
    double sign = (bytes[i] & 0x80) ? -1.0 : 1.0;
    int mag = bytes[i] & 0x7f;
    setMotorTarget(i, sign * maxRpm * mag / 127.0);
  }
  armWatchdog();
}

// Parses a number argument; prints an error and returns false if it isn't one
bool parseNumber(const char* text, double* out, const char* errWord) {
  if (!text) {
    Serial.println(errWord);
    return false;
  }
  char* end;
  *out = strtod(text, &end);
  if (*end != '\0') {
    Serial.println(errWord);
    return false;
  }
  return true;
}

/*
  Text commands (replies are single words so the Serial Plotter doesn't graph them):
    S <motor> <rpm>           e.g. S 0 40 sets motor 0 to 40 rpm (clamped to maxRpm)
    set_speed <motor> <rpm>   long form of S
    stop
    set <name> <value> / get <name> / params
    plot on | plot off
    status
*/
void handleTextCommand(char* line) {
  char* cmd = strtok(line, " \t\r");
  if (!cmd) {
    return;  // blank line, e.g. a leftover newline
  }

  if (strcmp(cmd, "S") == 0 || strcmp(cmd, "set_speed") == 0) {
    char* a = strtok(NULL, " \t\r");
    char* b = strtok(NULL, " \t\r");
    if (!a || !b) {
      Serial.println("ERR_usage_S_MOTOR_RPM");
      return;
    }

    char* end;
    long m = strtol(a, &end, 10);
    if (*end != '\0' || m < 0 || m > 3) {
      Serial.println("ERR_motor_must_be_0_to_3");
      return;
    }

    double rpm;
    if (!parseNumber(b, &rpm, "ERR_bad_rpm")) {
      return;
    }

    hostCmdArmed = false;  // manual control: no watchdog
    setMotorTarget((int)m, rpm);
    Serial.printf("OK_motor%ld_rpm%ld%s\n", m, lround(rpm), fabs(rpm) > maxRpm ? "_clamped" : "");
  }
  else if (strcmp(cmd, "stop") == 0) {
    hostCmdArmed = false;
    stopAllMotors();
    Serial.println("OK_stopped");
  }
  else if (strcmp(cmd, "set") == 0) {
    char* a = strtok(NULL, " \t\r");
    char* b = strtok(NULL, " \t\r");
    if (!a || !b) {
      Serial.println("ERR_usage_set_NAME_VALUE");
      return;
    }
    double value;
    if (!parseNumber(b, &value, "ERR_bad_value")) {
      return;
    }
    const char* err = setParam(a, value);
    if (err) {
      Serial.println(err);
    } else {
      Serial.printf("OK_set_%s_%g\n", a, value);
    }
  }
  else if (strcmp(cmd, "get") == 0) {
    char* a = strtok(NULL, " \t\r");
    if (!a || !printParam(a)) {
      Serial.println("ERR_unknown_parameter");
    }
  }
  else if (strcmp(cmd, "params") == 0) {
    printParams();
  }
  else if (strcmp(cmd, "floor") == 0) {  // shorthand kept from the first version
    char* a = strtok(NULL, " \t\r");
    double value;
    if (!parseNumber(a, &value, "ERR_usage_floor_PWM")) {
      return;
    }
    const char* err = setParam("floor", value);
    if (err) {
      Serial.println(err);
    } else {
      Serial.printf("OK_floor_%d\n", minPwm);
    }
  }
  else if (strcmp(cmd, "plot") == 0) {
    char* a = strtok(NULL, " \t\r");
    if (a && strcmp(a, "on") == 0) {
      plotMode = true;
      Serial.println("OK_plot_on");
    } else if (a && strcmp(a, "off") == 0) {
      plotMode = false;
      Serial.println("OK_plot_off");
    } else {
      Serial.println("ERR_usage_plot_on_or_off");
    }
  }
  else if (strcmp(cmd, "status") == 0) {
    Serial.printf("STATUS mode=%s watchdog=%s floor=%d bad_checksums=%lu",
                  plotMode ? "plot" : "host", hostCmdArmed ? "armed" : "idle", minPwm, badChecksums);
    Serial.print(" targets=");
    for (int i = 0; i < 4; i++) {
      Serial.printf("%s%.1f", i > 0 ? "," : "", directions[i] * desiredSpeeds[i]);
    }
    Serial.print(" measured=");
    for (int i = 0; i < 4; i++) {
      Serial.printf("%s%.1f", i > 0 ? "," : "", signedRpms[i]);
    }
    Serial.print(" counts=");
    for (int i = 0; i < 4; i++) {
      Serial.printf("%s%ld", i > 0 ? "," : "", (long)forwardCounts(i));
    }
    Serial.println();
  }
  else {
    Serial.println("ERR_unknown_command");
  }
}

void handleMotorControl() {
  //V_L1,V_L2,V_R1,V_r2
  for (int i = 0; i < 4; i++) {

    // open loop / dead reckoning: no PID, command goes straight to the motor
    if (openLoop) {
      pidControllers[i]->SetMode(PID::Mode::Manual);
      writeMotor(i, directPwm[i]);
      continue;
    }

    if (desiredSpeeds[i] < 0.001) {
      // stopped: hold the PID in manual so it restarts cleanly from the output floor
      pidControllers[i]->SetMode(PID::Mode::Manual);
      controlSpeeds[i] = 0;
      writeMotor(i, 0);
    } else {
      pidControllers[i]->SetMode(PID::Mode::Automatic);  // no-op if already running
      pidControllers[i]->Compute();
      writeMotor(i, controlSpeeds[i] * directions[i]);
    }
  }
}

void handleCommand() {
  if (!Serial.available()) {
    return;
  }

  int first = Serial.peek();

  // Binary host commands: handle one packet and stop here
  if (first == CMD_HEADER) {
    Serial.read();
    handleRpmCommand();
    return;
  }
  if (first == LEGACY_CMD_HEADER) {
    Serial.read();
    handleVelocitiesCommand();
    return;
  }

  // Text command, one line
  char line[64];
  size_t len = Serial.readBytesUntil('\n', line, sizeof(line) - 1);
  line[len] = '\0';
  handleTextCommand(line);
}

// Target vs measured rpm per motor, in Serial Plotter format (plotter mode only)
void plotSpeeds() {
  if (!plotMode) {
    return;
  }
  unsigned long currentTime = millis();
  if (currentTime - lastPlotTime < PLOT_INTERVAL_MS) {
    return;
  }
  lastPlotTime = currentTime;

  for (int i = 0; i < 4; i++) {
    double target = openLoop ? directPwm[i] / 255.0 * maxRpm
                             : directions[i] * desiredSpeeds[i];
    Serial.printf("%starget%d:%.1f,measured%d:%.1f", i > 0 ? "," : "", i, target, i, signedRpms[i]);
  }
  Serial.println();
}

// Binary report for the host (host mode only):
//   'M', REPORT_PAYLOAD_LEN, 4 x (int16 LE rpm * RPM_SCALE, int32 LE counts), XOR checksum
void sendReport() {
  if (plotMode) {
    return;
  }
  unsigned long currentTime = millis();
  if (currentTime - lastReportTime < (unsigned long)reportMs) {
    return;
  }
  lastReportTime = currentTime;

  uint8_t pkt[2 + REPORT_PAYLOAD_LEN + 1];

  // If nothing is draining the port (no host attached) the TX buffer fills up and a write
  // would block the loop, so skip this report instead
  if (Serial.availableForWrite() < (int)sizeof(pkt)) {
    return;
  }

  pkt[0] = REPORT_HEADER;
  pkt[1] = REPORT_PAYLOAD_LEN;

  uint8_t* p = &pkt[2];
  for (int i = 0; i < 4; i++) {
    long rpmScaled = lround(signedRpms[i] * RPM_SCALE);
    if (rpmScaled > 32767) rpmScaled = 32767;
    if (rpmScaled < -32768) rpmScaled = -32768;
    uint16_t r = (uint16_t)(int16_t)rpmScaled;
    uint32_t c = (uint32_t)forwardCounts(i);

    p[0] = r & 0xFF;
    p[1] = (r >> 8) & 0xFF;
    p[2] = c & 0xFF;
    p[3] = (c >> 8) & 0xFF;
    p[4] = (c >> 16) & 0xFF;
    p[5] = (c >> 24) & 0xFF;
    p += 6;
  }

  uint8_t cs = 0;
  for (int k = 0; k < REPORT_PAYLOAD_LEN; k++) {
    cs ^= pkt[2 + k];
  }
  pkt[2 + REPORT_PAYLOAD_LEN] = cs;

  Serial.write(pkt, sizeof(pkt));
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);

  for (int i = 0; i < 8; i++) {
    pinMode(motorPins[i], OUTPUT);
    analogWrite(motorPins[i], 0);
  }

  for (int i = 0; i < 4; i++) {
    // Pull-ups cover open-collector Hall outputs; harmless if the encoder board has its own
    pinMode(encPins[i][0], INPUT_PULLUP);
    pinMode(encPins[i][1], INPUT_PULLUP);
    encStates[i] = (digitalRead(encPins[i][0]) << 1) | digitalRead(encPins[i][1]);
    attachInterruptArg(encPins[i][0], onEncoderEdge, (void*)(intptr_t)i, CHANGE);
    attachInterruptArg(encPins[i][1], onEncoderEdge, (void*)(intptr_t)i, CHANGE);

    pidControllers[i]->SetOutputLimits(minPwm, 255);
    pidControllers[i]->SetSampleTime(sampleMs);
    pidControllers[i]->SetMode(PID::Mode::Manual);
  }

  Serial.println("READY");
}

void loop() {
  handleCommand();
  checkWatchdog();
  updateSpeedMeasurements();
  handleMotorControl();
  plotSpeeds();
  sendReport();
  delay(1);
}
