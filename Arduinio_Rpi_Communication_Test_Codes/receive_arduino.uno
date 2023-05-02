void setup() {
  Serial.begin(9600);
}

void loop() {
  // Check if there is any data available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming data
    char incomingData = Serial.read();
    
    // Print the incoming data
    Serial.print(incomingData);
  }
}
