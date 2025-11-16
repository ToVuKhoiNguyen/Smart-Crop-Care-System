// Define the pin numbers
const int buttonPin = 13;  // D13
const int ledPin = 25;     // D25

// Variable to keep track of the LED state
volatile bool ledState = LOW;

// Interrupt service routine (ISR)
void IRAM_ATTR buttonISR() {
  ledState = !ledState;  // Toggle the LED state
  digitalWrite(ledPin, ledState);  // Update the LED
}

void setup() {
  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);
  
  // Initialize the button pin as an input with pull-up resistor
  pinMode(buttonPin, INPUT);
  
  // Attach the interrupt to the button pin
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
}

void loop() {
  // The loop is empty because the interrupt handles the button press
}