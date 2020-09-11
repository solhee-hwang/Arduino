#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(9600); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  Serial.println(++count);
  toggle = toggle_state(toggle); //toggle LED value.
  toggle += 1;
  digitalWrite(PIN_LED, toggle); // update LED status.
  delay(500); // wait for 1,000 milliseconds
  toggle -= 1;
  digitalWrite(PIN_LED, toggle);
  delay(500); // wait for 1,000 milliseconds
}

int toggle_state(int toggle) {
  return toggle;
}
