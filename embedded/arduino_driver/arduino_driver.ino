
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

// R1+, R1-, R2+, R2-
const int RIGHT_PINS[] = {D0, D1, D2, D3};
// const int RIGHT_PINS[] = {0, 1, 2, 3};

// L1+, L1-, L2+, L2-
const int LEFT_PINS[] = {D4, D5, D6, D7};
// const int LEFT_PINS[] = {4, 5, 6, 7};

bool isValidFloat(String str) {
  float num = str.toFloat();
  return !(num == 0.0 && str != "0" && str != "0.0"); // Ensure it's not false-positive
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    analogWrite(RIGHT_PINS[i], 0);
    analogWrite(LEFT_PINS[i], 0);
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
    SET_VS V_L1,V_L2,V_R1,V_r2

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
  writePair(LEFT_PINS[0], LEFT_PINS[1], velocities[0]); // L1
  writePair(LEFT_PINS[2], LEFT_PINS[3], velocities[1]); // L2
  writePair(RIGHT_PINS[0], RIGHT_PINS[1], velocities[2]); // R1
  writePair(RIGHT_PINS[2], RIGHT_PINS[3], velocities[3]); // R2
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
  for (int i = 0; i < 4; i++) {
    pinMode(RIGHT_PINS[i], OUTPUT);
    analogWrite(RIGHT_PINS[i], 0);

    pinMode(LEFT_PINS[i], OUTPUT);
    analogWrite(LEFT_PINS[i], 0);
  }
  analogWriteRange(255); // Set PWM range to 0-255
  Serial.begin(115200); // ESP8266 prefers higher baud rates

  Serial.println("Started and listening for commands");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    handleCommand(command);
  }
}