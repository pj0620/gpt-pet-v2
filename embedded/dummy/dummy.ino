void setup() {
  // start serial port at 9600 bps and wait for port to open:
  Serial.begin(9600);
}

void loop() {
  // if we get a valid byte, read analog ins:
  if (Serial.available()) {
    String command = Serial.readString();
    Serial.print("recieved: ");
    Serial.println(command);
  }
}
