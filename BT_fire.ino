#include<SoftwareSerial.h>
#include<Servo.h>
Servo MG995_Servo;
SoftwareSerial BTserial(10,11);
// for light
#define RED_CIRCLE  "A"
#define BLUE_CIRCLE "B"
#define GREEN_CIRCLE "C"
#define BLUE_CROSS_1 "D"
#define BLUE_CROSS_2 "E"
#define RED_TRI_1 "F"
#define RED_TRI_2 "G"
#define RED_TRI_3 "H"
#define LIGHT_OFF "I"
// for control
#define FIRE_ON 1
#define WATER_ON 2
#define CAR_FRONT 3
#define CAR_BACK 4
#define CAR_LEFT 5
#define CAR_RIGHT 6
#define CAR_OFF 7
//return state
#define RED_CIRCLE_D  "a"
#define BLUE_CIRCLE_D "b"
#define GREEN_CIRCLE_D "c"
#define BLUE_CROSS_1_D "d"
#define BLUE_CROSS_2_D "e"
#define RED_TRI_1_D "f"
#define RED_TRI_2_D "g"
#define RED_TRI_3_D "h"
#define LIGHT_OFF_D "i"
#define FIRE_ON_D "j"
#define WATER_ON_D "k"
#define CAR_OFF_D "l"
const int In1 = 4;
const int In2 = 5;
const int In3 = 6;
const int In4 = 7;
void setup() {
  // put your setup code here, to run once:
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  Serial.begin(9600);
  BTserial.begin(9600);
  MG995_Servo.attach(9);

}

void loop() {
  // put your main code here, to run repeatedly:
  while(BTserial.available()>0){
    char buf[100];
    int len = BTserial.readBytes(buf,100);
    if(buf[0] == 'E'){
      digitalWrite(In1,HIGH);
      BTserial.write(BLUE_CROSS_2_D);
    }
    else if(buf[0] == 'G'){
      digitalWrite(In2,HIGH);
      BTserial.write(RED_TRI_2_D);
    }
    else if(buf[0] == 'H'){
      digitalWrite(In3,HIGH);
      BTserial.write(RED_TRI_3_D);
    }
    else if(buf[0] == 'I'){
      digitalWrite(In1,LOW);
      digitalWrite(In2,LOW);
      digitalWrite(In3,LOW);
      BTserial.write(LIGHT_OFF_D);
    }
    else if(buf[0] == 1){
      MG995_Servo.write(135);
      delay(1000);
      MG995_Servo.write(90);
      BTserial.write(FIRE_ON_D);
    }
  }
}
