void setup() {
  pinMode(A7, INPUT);
  Serial.begin(9600);

}

void loop() {
  Serial.println(analogRead(A7));
  delay(1000);
}
