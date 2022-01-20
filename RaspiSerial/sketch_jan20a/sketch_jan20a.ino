String data;
int speed;
int direction;


void setup() {
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    
    data = Serial.readStringUntil('\n');
    speed     = data.substring(0,3).toInt();
    direction = data.substring(3,6).toInt();
    
    Serial.print('(');
    Serial.print(speed);
    Serial.print(',');
    Serial.print(direction);
    Serial.println(')');
  }
}
