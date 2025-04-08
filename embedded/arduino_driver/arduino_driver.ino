/*
  ## Motor Diagram: 
  ^
  | Forward 

  L1 +------+ R1
     |      |
     |      |
     |      |
  L2 +------+ R2

  ## The code below creates a command interface over serial with commands of the following form
  
  > SET_VS V_L1,V_L2,V_R1,V_r2

  - Where V_M sets the velocity of each motor in the range [-255, 255]
*/

// #include <WiFi.h>
// #include <ESPmDNS.h>
// #include <ESPAsyncWebServer.h>
// #include <Preferences.h>
#include <PID_v2.h>

#define MAX_ENC_INTERVAL 1000
#define MIN_ENC_INTERVAL 120
#define ENC_WINDOW_SIZE 20
#define SPEED_PUBLISH_INTERVAL 100
#define MAX_SPEED 2

// L1+, L1-, L2+, L2-, R1+, R1-, R2+, R2-
const int motorPins[] = {15, 16, 17, 18, 4, 5, 6, 7};

double encMult = 152.35;
double kps[4] = { 80,  80,  80,   80 };
double kis[4] = { 120, 120, 120, 120 };
double kds[4] = { 2,   2,   2,   2   };

// L1, L2, R1, R2
int encPins[4] = {
  10, 11, 12, 13
};
unsigned long times[4] = {
  0, 0, 0, 0
};
int encLevels[4] = {
  200, 300, 300, 300
};

int envNegWindow[4][ENC_WINDOW_SIZE];
int envNegVals[4] = {0, 0, 0, 0};
int envPosWindow[4][ENC_WINDOW_SIZE];
int envPosVals[4] = {0, 0, 0, 0};
int envWindowIdx = 0;

// v_L1, v_L2, v_R1, v_R2
static double measuredSpeeds[4] = {
  0, 0, 0, 0
};
static double desiredSpeeds[4] = {
  0, 0, 0, 0
};

// 1 -> forward, -1 -> reverse
double directions[4] = {
  1, 1, 1, 1
};

static double controlSpeeds[4] = {
  0, 0, 0, 0
};

PID l1Pid(&measuredSpeeds[0], &controlSpeeds[0], &desiredSpeeds[0], kps[0], kis[0], kds[0], DIRECT);
PID l2Pid(&measuredSpeeds[1], &controlSpeeds[1], &desiredSpeeds[1], kps[1], kis[1], kds[1], DIRECT);
PID r1Pid(&measuredSpeeds[2], &controlSpeeds[2], &desiredSpeeds[2], kps[2], kis[2], kds[2], DIRECT);
PID r2Pid(&measuredSpeeds[3], &controlSpeeds[3], &desiredSpeeds[3], kps[3], kis[3], kds[3], DIRECT);
static PID* pidControllers[4] = { &l1Pid, &l2Pid, &r1Pid, &r2Pid};

unsigned long lastPublishTime = 0;

void updateSpeedMeasurements() {
  unsigned long curTime = millis();
  for (int i = 0; i < 4; i++) {
    // current sensor measurement
    int encPin = encPins[i];
    int sensorValue = analogRead(encPin);

    // update slidding windoes for rising edge detection
    envPosVals[i] += sensorValue;
    int lastEnvPosVal = envPosWindow[i][envWindowIdx];
    envPosVals[i] -= lastEnvPosVal;
    envPosWindow[i][envWindowIdx] = sensorValue;
    
    int lastEnvNegVal = envNegWindow[i][envWindowIdx];
    envNegVals[i] += lastEnvPosVal;
    envNegVals[i] -= lastEnvNegVal;
    envNegWindow[i][envWindowIdx] = lastEnvPosVal;

    // time difference to last pulse
    unsigned long lastTime = times[i];
    unsigned long delTime = curTime - lastTime;

    // rising edge is detected
    if ((envPosVals[i] - envNegVals[i])/ENC_WINDOW_SIZE > encLevels[i]) {
      
      // time difference in correct interval, update measured speed
      if (MIN_ENC_INTERVAL < delTime && delTime < MAX_ENC_INTERVAL) {
        measuredSpeeds[i] = encMult / delTime;
      }

      // set last rising interval time, no checking for MAX_ENC_INTERVAL to handle motor stopping
      if (MIN_ENC_INTERVAL < delTime) {
        times[i] = curTime;
      }
    }

    // we havent seen a pulse in MAX_ENC_INTERVAL, assume motor has stopped 
    if (delTime > MAX_ENC_INTERVAL) {
      measuredSpeeds[i] = 0;
    }
  }
  envWindowIdx = (envWindowIdx + 1) % ENC_WINDOW_SIZE;
}

bool isValiddouble(String str) {
  double num = str.toDouble();
  return !(num == 0.0 && str != "0" && str != "0.0"); // Ensure it's not false-positive
}

double sgn(double x) {
  return (double) ((x > 0) - (x < 0));
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

/*
  Processing commands of the following form: 
    SET_VS V_L1,V_L2,V_R1,V_R2

  Where V_x is the velocity of each motor in the range [-255,255]
*/
void handleVelocitiesCommand() {
  for (int i = 0; i < 4; i++) {
    int inpVel = Serial.read();
    if (inpVel & 0x80) {
      Serial.print("-");
      directions[i] = -1.0;
    } else {
      Serial.print("+");
      directions[i] = 1.0;
    }
    desiredSpeeds[i] = MAX_SPEED * (inpVel & 0x7f) / 127.0;
  }
}

void handlePidControl() {
  for (int i = 0; i < 4; i++) {

    //V_L1,V_L2,V_R1,V_r2
    int posPin = motorPins[2 * i];
    int negPin = motorPins[2 * i + 1];

    if (-0.001 < desiredSpeeds[i] && desiredSpeeds[i] < 0.001) {
      controlSpeeds[i] = 0;
      writePair(posPin, negPin, 0);
    } else {
      pidControllers[i]->Compute();
      // Serial.printf("i=%d direction=%f control=%f measured=%f desired=%f posPin=%d negPin=%d", i, directions[i], controlSpeeds[i] * directions[i], measuredSpeeds[i], desiredSpeeds[i], posPin, negPin);
      // Serial.println("");
      writePair(posPin, negPin, controlSpeeds[i] * directions[i]);
    }
  }
}

void handleCommand() {
  if (!Serial.available()) {
    return;
  }

  char incomingByte = Serial.read();

  if (incomingByte == 'V') {
    handleVelocitiesCommand();
  }

  else {
    Serial.println("invalid command!");
  }

  // Flush input
  while (Serial.available()) {
    Serial.read();
  }

  Serial.println("DONE");
}

void writeSpeeds() {
  auto currentTime = millis();
  if (currentTime - lastPublishTime < SPEED_PUBLISH_INTERVAL) {
    return;
  }
  lastPublishTime = currentTime;

  Serial.print('M');
  for (int i=0; i < 4; i++) {

    char val = char(round(127 * measuredSpeeds[i] / MAX_SPEED));

    if (val > 127) {
      val = 127;
    }

    if (directions[i] < 0) {
      val = val | 0x80;
    }

    Serial.print(val);
  }
  Serial.println("");
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 8; i++) {
    pinMode(motorPins[i], OUTPUT);
    analogWrite(motorPins[i], 0);
  }

  for (int i = 0; i < 4; i++) {
    pinMode(encPins[i], INPUT);
    pidControllers[i]->SetOutputLimits(75, 255);
    pidControllers[i]->SetMode(PID::Mode::Automatic);
  }

  Serial.println("READY");
}

void loop() {
  handleCommand();
  updateSpeedMeasurements();
  handlePidControl();
  writeSpeeds();
  delay(1);
}