#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  count = toggle = 1;
  digitalWrite(PIN_LED, toggle); 
}

void loop() {
  Serial.println(++count);
  toggle = toggle_state(toggle);
  digitalWrite(PIN_LED, toggle);
  delay(1000);
  for (int i=1; i <=11; i++) {
      Serial.println(++count);
      toggle = toggle_state(toggle);
      digitalWrite(PIN_LED, toggle);
      delay(100);
      Serial.println(++count);
  }   
  while(1){}
}

int toggle_state(int toggle) {
  return 1 - toggle;
}
