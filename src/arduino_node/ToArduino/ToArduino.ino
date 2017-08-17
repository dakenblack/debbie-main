#include <SoftwareSerial.h>
#include <Sabertooth.h>

SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.

#define LEFT_MOTOR 1
#define RIGHT_MOTOR 2

#define MAX_VAL 40
int abs(int a) {
  return a < 0 ? -1*a : a; 
}

int _ceil(int a, int m) {
  if(a < 0) {
    return a < -1 * m ? -1 * m : a;
  } else {
    return a > m ? m : a;
  }
}

static unsigned char* deserialize_int(unsigned char* buf, signed int *a) {
    *a = 0;
    *a = (*buf++ << 8) | *a;
    *a = (*buf++) | *a;
    return buf;
}

void left_move(int a);
void right_move(int a);

void setup()
{
  SWSerial.begin(9600);
  Serial.begin(9600);
  ST.autobaud();
}

void loop()
{
	unsigned char buffer[6];
  //Serial.println(Serial.available());
	if(Serial.available() >= 6) {
		for(int i=0; i<6; i++) {
			buffer[i] = (unsigned char)Serial.read();
		}
   
		signed int salt,left,right;
		unsigned char* ptr = buffer;
		ptr = deserialize_int(ptr, &salt);
		ptr = deserialize_int(ptr, &left);
		ptr = deserialize_int(ptr, &right);
		
		if(salt != 1337) {
			Serial.print("a");
			return;
		}
		left = map(left,-115,115,-40,40);
   right = map(right,-115,115,-40,40);
		left_move(left);
		right_move(right);
    Serial.println("b");
	}
}


void left_move(int a) {
  Serial.print("left ");
  Serial.print(a);
	ST.motor(LEFT_MOTOR,a);
}
void right_move(int a) {
  Serial.print(" right ");
  Serial.println(a);
	ST.motor(RIGHT_MOTOR,a);
}
