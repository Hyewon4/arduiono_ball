void setup() {
  // put your setup code here, to run once:
  pinMode(7, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(7, 0);
  delay(1000);
  for(int i = 0; i < 5; i++){
    digitalWrite(7, 1);
    delay(100);
    digitalWrite(7, 0);
    delay(100);
  }
  while(1){
    digitalWrite(7, 1);
  }
}
