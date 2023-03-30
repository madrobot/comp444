int potPosition;            // This variable will hold a value based on the position of the potentiometer

void setup() {
  Serial.begin(9600);       // Start a serial connection with the computer
  pinMode(13, OUTPUT);      // Set pin 13 as an output that can be set to HIGH or LOW
}

void loop() {
  // Rad the position of the pot
  potPosition = analogRead(A0);    // Set potPosition to a number between 0 and 1023 based on how far the knob is turned
  Serial.println(potPosition);     // Print the value of potPosition in the serial monitor on the computer

  // Change the LED blink speed based on the pot value
  digitalWrite(13, HIGH);          // Turn on the LED
  delay(potPosition);              // Delay for as many milliseconds as potPosition (0-1023)

  digitalWrite(13, LOW);           // Turn off the LED
  delay(potPosition);              // Delay for as many milliseconds as potPosition (0-1023)
}
