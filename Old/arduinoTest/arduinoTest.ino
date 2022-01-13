#include <Servo.h>

#define SERVO 0
#define THROTTLE 1

int servo_rx = 2;
int throttle_rx = 3;

Servo servo_control;
Servo throttle_control;

bool throttle_high = false;
bool servo_high = false;


unsigned long start_time = 0;
unsigned long high_time = 0;

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


int controlVal = 1000;

void loop() {
		delay(200);
		Serial.println(controlVal);
		throttle_control.writeMicroseconds(controlVal);

		controlVal += 10;

		if (controlVal >= 2000) {
			controlVal = 1000;
		}
}

