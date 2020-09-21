
#define PIN_LED 7
unsigned int count, toggle, led_count;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(9600); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = led_count = 0;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  led_count = 1;
  toggle = toggle_state(toggle); //toggle LED value.
  digitalWrite(PIN_LED, toggle); // update LED status.
  delay(1000);
  for(;led_count<=10;led_count++){
   toggle = toggle_state(toggle); //toggle LED value.
   digitalWrite(PIN_LED, toggle); // update LED status.
   delay(100); 
  }
  led_count = 1;
  while(1){}
}

int toggle_state(int toggle) {
   if (led_count%2==1){
      toggle = 0;
   }else{
      toggle = 1;
   }
   return toggle;
}
