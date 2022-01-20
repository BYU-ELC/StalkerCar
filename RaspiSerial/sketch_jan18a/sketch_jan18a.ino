int speed;
int direction;
String incomingString;

void serialFlush() {
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}

void setup() {
  Serial.begin(9600);
  serialFlush();
}

void loop() {
  while (!Serial.available()) {}
  incomingString = Serial.readStringUntil('\n');
  speed = incomingString.substring(0,3).toInt();
  direction = incomingString.substring(3, 6).toInt();

  Serial.print('(');
  Serial.print(speed);
  Serial.print(',');
  Serial.print(direction);
  Serial.println(')');
}
