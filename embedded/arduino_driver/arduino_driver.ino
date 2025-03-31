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

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <PID_v2.h>

#define ENC_LEVEL 500
#define MAX_ENC_INTERVAL 1000
#define MIN_ENC_INTERVAL 140
#define ENC_WINDOW_SIZE 20

// R1+, R1-, R2+, R2-
const int rightMotorPins[] = {4, 5, 6, 7};

// L1+, L1-, L2+, L2-
const int leftMotorPins[] = {15, 16, 17, 18};

float encMult = 152.35;
float kp = 100.0;
float ki = 0;
float kd = 0;

// R1, nan, nan, nan
int encPins[4] = {
  10, 11, 12, 13
};
unsigned long times[4] = {
  0, 0, 0, 0
};

int envNegWindow[4][ENC_WINDOW_SIZE];
int envNegVals[4] = {0, 0, 0, 0};
int envPosWindow[4][ENC_WINDOW_SIZE];
int envPosVals[4] = {0, 0, 0, 0};
int envWindowIdx = 0;

// v_L1, v_L2, v_R1, v_R2
float measuredSpeeds[4] = {
  0, 0, 0, 0
};
float desiredSpeeds[4] = {
  0, 0, 0, 0
};

// 1 -> forward, -1 -> reverse
float directions[4] = {
  0, 0, 0, 0
};

float lastErrors[4] = {
  0, 0, 0, 0
};
float sumErrors[4] = {
  0, 0, 0, 0
};
float controlSpeeds[4] = {
  0, 0, 0, 0
};

unsigned long lastPIDUpdateTime = 0;

// Create PID controller instance
PID_v2 myPID(kp, ki, kd, DIRECT);

Preferences preferences;

const char *ssid = "HONEST.NET-1905_2G";
const char *password = "vuvohuge65";

char homepage[2048];
AsyncWebServer server(80);

void generateHomepage() {
  Serial.print("encMult = ");
  Serial.println(encMult);
  snprintf(homepage, sizeof(homepage), R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>GPTPet ESP Server</title>
  <style>
    body {
      background-color: black;
      color: white;
      font-family: sans-serif;
    }
    .form-row {
      display: flex;
      justify-content: space-between;
      margin-bottom: 10px;
    }
    label {
      flex: 1;
      text-align: left;
    }
    input {
      flex: 1;
      text-align: right;
    }
  </style>
</head>
<body>
  <h1>GPTPet ESP Control Server</h1>
  <form action="/submit" method="POST">
    <div class="form-row">
      <label for="kp">KP:</label>
      <input type="number" id="kp" name="kp" value="%f" required>
    </div>
    <div class="form-row">
      <label for="ki">KI:</label>
      <input type="number" id="ki" name="ki" value="%f" required>
    </div>
    <div class="form-row">
      <label for="kd">KD:</label>
      <input type="number" id="kd" name="kd" value="%f" required>
    </div>
    <div class="form-row">
      <label for="encnum">Encoder Numerator:</label>
      <input type="number" id="encnum" name="encnum" value="%f" required>
    </div>
    <button type="submit">Submit</button>
  </form>
</body>
</html>
)rawliteral", kp, ki, kd, encMult);
}

void updateSpeedMeasurements() {
  unsigned long curTime = millis();
  for (int i = 0; i < 4; i++) {
    // current sensor measurement
    int encPin = encPins[i];
    int sensorValue = analogRead(encPin);

    // measuredSpeeds[i] = sensorValue;

    // update slidding windoes for rising edge detection
    envPosVals[i] += sensorValue;
    int lastEnvPosVal = envPosWindow[i][envWindowIdx];
    envPosVals[i] -= lastEnvPosVal;
    envPosWindow[i][envWindowIdx] = sensorValue;
    
    int lastEnvNegVal = envNegWindow[i][envWindowIdx];
    envNegVals[i] += lastEnvPosVal;
    envNegVals[i] -= lastEnvNegVal;
    envNegWindow[i][envWindowIdx] = lastEnvPosVal;

    // measuredSpeeds[i] = (envPosVals[i] - envNegVals[i]) / ENC_WINDOW_SIZE;

    // time difference to last pulse
    unsigned long lastTime = times[i];
    unsigned long delTime = curTime - lastTime;

    // rising edge is detected
    if ((envPosVals[i] - envNegVals[i])/ENC_WINDOW_SIZE > ENC_LEVEL) {
      
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

void submitCallback(AsyncWebServerRequest *request) {
  Serial.println("/submit request recieved");
  if (!(
    request->hasParam("kp", true) &&
    request->hasParam("ki", true) &&
    request->hasParam("kd", true) &&
    request->hasParam("encnum", true)
  )) {
    request->send(400, "text/plain", "invalid request!");
  }
  kp = atof(request->getParam("kp", true)->value().c_str());
  Serial.print("kp = ");
  Serial.println(kp);

  ki = atof(request->getParam("ki", true)->value().c_str());
  Serial.print("ki = ");
  Serial.println(ki);

  kd = atof(request->getParam("kd", true)->value().c_str());
  Serial.print("kd = ");
  Serial.println(kd);

  encMult = atof(request->getParam("encnum", true)->value().c_str());
  Serial.print("encMult = ");
  Serial.println(encMult);

  preferences.putFloat("kp", kp); 
  preferences.putFloat("ki", ki); 
  preferences.putFloat("kd", kd); 
  preferences.putFloat("encMult", encMult); 

  generateHomepage();

  request->send(200, "text/plain", "Updated!");
}

// void updateVelocities() {
//   unsigned long now = millis();
//   double timeChange = (double)(now - lastPIDUpdateTime);
//   for (int i = 0; i < 4; i++) {
//     double error = desiredSpeeds[i] - measuredSpeeds[i];
//     sumErrors[i] += error * timeChange;
//     double dErr = (error - lastErrors[i]) / timeChange;

//     double rawOutput = kp * error + ki * sumErrors[i] + kd * dErr;

//     //TODO: 
//     Serial.print(rawOutput);
//     Serial.print(" ");

//     lastErrors[i] = error;
//   }
//   Serial.println("");
//   lastPIDUpdateTime = now;
// }

bool isValidFloat(String str) {
  float num = str.toFloat();
  return !(num == 0.0 && str != "0" && str != "0.0"); // Ensure it's not false-positive
}

int sgn(int x) {
  return (x > 0) - (x < 0);
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    analogWrite(rightMotorPins[i], 0);
    analogWrite(leftMotorPins[i], 0);
  }
}

void writePair(int positivePin, int negativePin, float value) {
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
void handleVelocitiesCommand(String command) {
  Serial.println("Processing velocities command");
  String noHeaderCmd = command.substring(7);

  // Parse Velocities
  int velocities[4];
  int lastCommaIdx = 0;
  int commaIdx;
  String vel;
  for (int i = 0; i < 4; i++) {
    if (i == 0) {
      commaIdx = noHeaderCmd.indexOf(",");
      vel = noHeaderCmd.substring(0, commaIdx);
    }
    else if (0 < i && i < 3) {
      commaIdx = noHeaderCmd.indexOf(",", lastCommaIdx);
      vel = noHeaderCmd.substring(lastCommaIdx, commaIdx);
    } 
    else {
      commaIdx = noHeaderCmd.indexOf(",", lastCommaIdx);
      if (commaIdx != -1) {
        Serial.println("Invalid number of arguments. Must provide 4 comma seperated values");
        return;
      }
      vel = noHeaderCmd.substring(lastCommaIdx, noHeaderCmd.length());
    }

    if (commaIdx == -1 && i < 3) {
      Serial.println("Invalid number of arguments. Must provide 4 comma seperated values");
      return;
    }

    velocities[i] = vel.toFloat();
    if (!(velocities[i] == 0.0 && vel != "0" && vel != "0.0")) {
      if (velocities[i] < -255.0 || velocities[i] > 255.0) {
        Serial.print("invalid velocity recieved (must be in range [-255, 255]) ");
        Serial.print(vel);
        Serial.println("!");
        return;
      }
    } else {
      Serial.print("invalid velocity recieved (not a float) ");
      if (vel.length() == 0) {
        Serial.print("<EMPTY STRING>");
      } else {
        Serial.print(vel);
      }
      Serial.println("!");
      return;
    }
    lastCommaIdx = commaIdx + 1;
  }

  // Write to pins 
  // writePair(leftMotorPins[0], leftMotorPins[1], velocities[0]); // L1
  // writePair(leftMotorPins[2], leftMotorPins[3], velocities[1]); // L2
  // writePair(rightMotorPins[0], rightMotorPins[1], velocities[2]); // R1
  // writePair(rightMotorPins[2], rightMotorPins[3], velocities[3]); // R2

  for (int i = 0; i < 4; i++) {
    desiredSpeeds[i] = abs(velocities[i]);
    directions[i] = sgn(directions[i]);
  }

  // // myPID.SetPoint(desiredSpeeds[2]);
  // myPID.Start(controlSpeeds[2],       // input
  //             measuredSpeeds[2],      // current output
  //             desiredSpeeds[2]);      // setpoint
}

void handleCommand(String command) {
  Serial.println("Processing command: " + command);

  if (command.startsWith("SET_VS ")) {
    handleVelocitiesCommand(command);
  }

  else {
    Serial.println("invalid command!");
  }

  Serial.println("DONE");
}

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Open Preferences with a namespace (e.g. "my-app"), RW mode
  preferences.begin("gptpet", false);
  kp = preferences.getFloat("kp", 1);
  ki = preferences.getFloat("ki", 0);
  kd = preferences.getFloat("kd", 0);
  encMult = preferences.getFloat("encMult", 304.7);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  delay(500);  // <-- Add this right after connection
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  for (int i = 0; i < 4; i++) {
    pinMode(rightMotorPins[i], OUTPUT);
    analogWrite(rightMotorPins[i], 0);

    pinMode(leftMotorPins[i], OUTPUT);
    analogWrite(leftMotorPins[i], 0);

    pinMode(encPins[i], INPUT);
  }
  Serial.begin(115200); // ESP8266 prefers higher baud rates

  generateHomepage();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("/ request recieved");
    request->send(200, "text/html", homepage);
  });
  server.on("/submit", HTTP_POST, submitCallback);

  server.begin();
  Serial.println("HTTP server started");

  // // Initialize the PID controller
  // myPID.SetMode(AUTOMATIC);
  // Output limited to valid PWM range (0-255)
  myPID.SetOutputLimits(0, 255);

  myPID.Start(0,       // input
              0,       // current output
              0);    // setpoint
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    handleCommand(command);
  }

  updateSpeedMeasurements();
  // updateVelocities();

  // Run the PID computation
  // myPID.Compute();
  // if (desiredSpeeds[2] > 1) {
  //   const double output = myPID.Run(measuredSpeeds[2]);
  //   analogWrite(rightMotorPins[0], output);
  // } 

  for (int i=0; i < 4; i++) {
    Serial.print(measuredSpeeds[i]);
    Serial.print(" ");
  }
  Serial.println(" -1000 1000");

  // myPID.Start(controlSpeeds[2],       // input
  //             measuredSpeeds[2],      // current output
  //             desiredSpeeds[2]);      // setpoint

  // Serial.print(controlSpeeds[2]);
  // Serial.print(" ");
  // Serial.print(measuredSpeeds[2]);
  // Serial.print(" ");
  // Serial.println(desiredSpeeds[2]);

  // for (int i = 0; i < 4; i++) {
  //   Serial.print(controlSpeeds[i] * directions[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  delay(1);
}