#include <Servo.h>

#define SERVO 0
#define THROTTLE 1

int servo_rx = 2;
int throttle_rx = 3;

Servo servo_control;
Servo throttle_control;

bool throttle_high = false;
bool servo_high = false;

bool control_type;

unsigned long start_time = 0;
unsigned long high_time = 0;

void manual_control(){
  servo_control.detach();
  throttle_control.detach();
  int servo_controller = 4;
  int throttle_controller = 5;
  pinMode(servo_controller, OUTPUT);
  pinMode(throttle_controller, OUTPUT);
  while(1){
    if(digitalRead(throttle_rx) && !throttle_high){
      digitalWrite(throttle_controller, HIGH);
      throttle_high = true;
    }
    else if(!digitalRead(throttle_rx) && throttle_high){
      digitalWrite(throttle_controller, LOW);
      throttle_high = false;
    }
    if(digitalRead(servo_rx) && !servo_high){
     digitalWrite(servo_controller, HIGH);
     servo_high = true;
    }
    else if(!digitalRead(servo_rx) && servo_high){
     digitalWrite(servo_controller, LOW);
     servo_high = false;
    }
  }
}

int convert_input(bool type, int input){
  if(type == SERVO){
    return (3*input + 1200);
  }
  else if(type == THROTTLE){
    return (5*input + 1000);
  }
  else{
    return -1;
  }
}
String myString;

void setup() {
  // put your setup code here, to run once:
  pinMode(servo_rx, INPUT);
  pinMode(throttle_rx, INPUT);
  servo_control.attach(4, 1200, 1800);
  throttle_control.attach(5, 1000, 2000);
  servo_control.writeMicroseconds(1500);
  throttle_control.writeMicroseconds(1500);
  delay(2000);
  Serial.begin(9600);
}

int val;

void loop() {
  if(digitalRead(servo_rx)){
    start_time = micros();
    while(digitalRead(servo_rx)){}
    high_time = micros() - start_time;
    if(high_time > 1700){
      manual_control(); 
    }
  }
  while(Serial.available() > 0){ 
    control_type = Serial.read();
    delay(20);
    if(control_type == SERVO){
      servo_control.writeMicroseconds(convert_input(SERVO, Serial.read()));
    }
    else if(control_type == THROTTLE){
      throttle_control.writeMicroseconds(convert_input(THROTTLE, Serial.read()));
    }
  }
}

