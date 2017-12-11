uint32_t i=0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(i);
  delay(1000);
  i++;
}
