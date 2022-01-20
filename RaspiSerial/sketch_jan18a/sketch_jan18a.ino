int speed;
int direction;
String incomingString;

void setup() {
  Serial.begin(9600);  
}

void loop() {
  //while (Serial.available()) {}
  //incomingString = Serial.readString();
  //speed = incomingString.substr(0,3);
  //direction = incomingString.substr(3);

  Serial.print('(');
  Serial.print(speed);
  Serial.print(',');
  Serial.print(direction);
  Serial.println(')');
}
