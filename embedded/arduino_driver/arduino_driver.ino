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
      reverse. Targets beyond +-MAX_RPM are clamped. A bad checksum drops the packet.
      This is what the ROS2 hardware interface (ros2_ws/src/bot_ws/src/gptpet_hardware) sends,
      50 times a second.
  - 'V' (legacy, binary): 'V' + 4 bytes, bit 7 = reverse, bits 0..6 = 0..127 -> 0..MAX_RPM.
  - anything else: a newline-terminated text command (Serial Monitor / Plotter input box):
      S <motor> <rpm>           motor 0..3 = L1, L2, R1, R2; rpm is signed, e.g. S 0 40 or S 3 -34
      set_speed <motor> <rpm>   long form of S
      stop                      all motors off
      plot on | plot off        Serial Plotter mode, see Output below. Default off.
      status                    one-line summary of mode, watchdog, targets and measurements

  ## Watchdog
  After a 'W' or 'V' command, if no further one arrives within HOST_CMD_TIMEOUT_MS all motors
  are stopped (prints WATCHDOG_stopped once). Text commands are not covered by it, so manual
  tuning from the Serial Monitor keeps running.

  ## Control modes (set with directWrite below)
  - directWrite = false: closed loop. Each command is a target speed and a PID per motor
    drives the PWM using encoder feedback.
  - directWrite = true: open loop (dead reckoning). Each command is mapped straight to PWM
    (0..MAX_RPM -> 0..255) and written to the motor; the PID is bypassed. Encoders are
    still read, but they don't affect the output.

  ## Output
  - Host mode (default): every REPORT_INTERVAL_MS a binary report, 27 bytes:
      'M', 24, then per motor (L1, L2, R1, R2): int16 measured rpm * 100 (signed, taken from
      the encoder so it is the real direction) + int32 encoder counts (+ is forward), then
      1 byte XOR checksum of the 24 payload bytes.
  - Plotter mode ('plot on'): every PLOT_INTERVAL_MS, target vs measured rpm per motor for the
    Arduino Serial Plotter, and no binary reports:
      target0:40.0,measured0:39.6,target1:0.0,measured1:0.0,...
    Replies to text commands are single words so the plotter doesn't graph them.
*/

// #include <WiFi.h>
// #include <ESPmDNS.h>
// #include <ESPAsyncWebServer.h>
// #include <Preferences.h>
#include <PID_v2.h>

#define MAX_RPM 76.0             // rated no-load speed at 12V; targets are clamped here
#define MIN_PWM 75               // PID output floor, so the motor doesn't sit in its deadband
#define SPEED_SAMPLE_MS 20       // speed measurement and PID period
#define REPORT_INTERVAL_MS 20    // binary 'M' report period (host mode)
#define PLOT_INTERVAL_MS 50      // Serial Plotter line period (plotter mode)
#define SERIAL_TIMEOUT_MS 20     // how long to wait for the rest of a command
#define HOST_CMD_TIMEOUT_MS 500  // stop the motors if the host goes quiet for this long

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

// true -> open loop: bypass PID, commands go straight to the motors as PWM (no feedback)
const bool directWrite = false;

// L1+, L1-, L2+, L2-, R1+, R1-, R2+, R2-
const int motorPins[] = {15, 16, 17, 18, 4, 5, 6, 7};

// Motors wired backwards (a positive command drove them in reverse). For these, the PWM sent
// to the driver and the measured speed sign are both flipped, so +rpm means forward on every
// motor and commands, targets and reported speeds all stay in that frame.
//                              L1(0)  L2(1)  R1(2)  R2(3)
const bool motorReversed[4] = { true,  true,  false, true };

// Starting gains for rpm units; tune per motor if needed
double kps[4] = { 3,  3,  3,  3  };
double kis[4] = { 15, 15, 15, 15 };
double kds[4] = { 0,  0,  0,  0  };

// Encoder pins per motor {A, B}: L1, L2, R1, R2
int encPins[4][2] = {
  {35, 37},
  {41, 38},
  {40, 42},
  {36, 39},
};

// Set to -1 for any motor whose measured speed has the opposite sign to its target
// (its encoder A/B wires are the other way round). Fix the sign here, not in motorReversed.
// Only affects the reported sign, not control.
int encSign[4] = { 1, 1, 1, 1 };

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

// open-loop PWM per motor in [-255, 255], only used when directWrite is true
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

// Converts encoder counts since the last sample into rpm, every SPEED_SAMPLE_MS
void updateSpeedMeasurements() {
  unsigned long nowUs = micros();
  unsigned long elapsedUs = nowUs - lastSpeedSampleUs;
  if (elapsedUs < SPEED_SAMPLE_MS * 1000UL) {
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

// Sets one motor's target from a signed rpm, clamped to +-MAX_RPM
void setMotorTarget(int i, double rpm) {
  if (rpm > MAX_RPM) rpm = MAX_RPM;
  if (rpm < -MAX_RPM) rpm = -MAX_RPM;

  directions[i] = (rpm < 0) ? -1.0 : 1.0;
  desiredSpeeds[i] = fabs(rpm);
  directPwm[i] = rpm / MAX_RPM * 255.0;
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
  if (millis() - lastHostCmdMs < HOST_CMD_TIMEOUT_MS) {
    return;
  }
  hostCmdArmed = false;
  bool wasMoving = anyTargetSet();
  stopAllMotors();
  if (wasMoving) {
    Serial.println("WATCHDOG_stopped");
  }
}

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
  Where bit 7 is the direction and bits 0..6 are the speed 0..127 (0..MAX_RPM)
*/
void handleVelocitiesCommand() {
  uint8_t bytes[4];
  if (Serial.readBytes(bytes, 4) != 4) {
    return;  // a missing byte used to be read as -1, i.e. full speed reverse
  }

  for (int i = 0; i < 4; i++) {
    double sign = (bytes[i] & 0x80) ? -1.0 : 1.0;
    int mag = bytes[i] & 0x7f;
    setMotorTarget(i, sign * MAX_RPM * mag / 127.0);
  }
  armWatchdog();
}

/*
  Text commands (replies are single words so the Serial Plotter doesn't graph them):
    S <motor> <rpm>           e.g. S 0 40 sets motor 0 to 40 rpm (clamped to MAX_RPM)
    set_speed <motor> <rpm>   long form of S
    stop
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

    double rpm = strtod(b, &end);
    if (*end != '\0') {
      Serial.println("ERR_bad_rpm");
      return;
    }

    hostCmdArmed = false;  // manual control: no watchdog
    setMotorTarget((int)m, rpm);
    Serial.printf("OK_motor%ld_rpm%ld%s\n", m, lround(rpm), fabs(rpm) > MAX_RPM ? "_clamped" : "");
  }
  else if (strcmp(cmd, "stop") == 0) {
    hostCmdArmed = false;
    stopAllMotors();
    Serial.println("OK_stopped");
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
    Serial.printf("STATUS mode=%s watchdog=%s bad_checksums=%lu",
                  plotMode ? "plot" : "host", hostCmdArmed ? "armed" : "idle", badChecksums);
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
    if (directWrite) {
      writeMotor(i, directPwm[i]);
      continue;
    }

    if (desiredSpeeds[i] < 0.001) {
      // stopped: hold the PID in manual so it restarts cleanly from MIN_PWM
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

  // Text command (S, set_speed, stop, plot, status), one line
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
    double target = directWrite ? directPwm[i] / 255.0 * MAX_RPM
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
  if (currentTime - lastReportTime < REPORT_INTERVAL_MS) {
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

    pidControllers[i]->SetOutputLimits(MIN_PWM, 255);
    pidControllers[i]->SetSampleTime(SPEED_SAMPLE_MS);
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
