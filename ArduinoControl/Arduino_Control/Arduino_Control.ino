//Control code for Arduino

#include <servo.h>

int degree;       
int velocity;
int oldDegree;
int oldVelocity;

int servoPin;
int motorPin;
int delayTime = 50;

Servo Turner;
Servo Motor;

void setup() {
  //set up steering servo and motor pins
  Turner.attach(servoPin);
  Motor.attach(motorPin);
}

void loop() {

  //receive serial here and store in degree and velocity 
  //degree between 0 and 180 ( > 90 right, < 90 left)
  //go between 0 and 180 (> 90 forward, < backward)

  if (oldDegree == degree && oldVelocity == velocity) { }
  else 
  {
    turn(degree);
    go(velocity);
    delay(delayTime);
  }

}

//Function to turn 
void turn(int degree)
{
  Turner.write(degree);
}


//Function to control motor
void go(int velocity)
{
  Motor.write(velocity);
}
