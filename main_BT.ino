#include<SoftwareSerial.h>

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
#define CAR_OVER 8
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
#define CAR_OVER_D "l"

void BT_light_rx();
void BT_lightandwater_rx();
void BT_fire_rx();
void BT_car_rx();
void BT_main_tx();

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  BTserial.begin(9600);

}

void loop()
{
  BT_main_tx();
  BT_light_rx();
  BT_lightandfire_rx();
  BT_water_rx();
  BT_car_rx();
}

void BT_light_rx(){
  while(Serial1.available()>0){
    char buf[100];
    int len = Serial1.readBytes(buf,100);
    if(buf[0] == 'a'){
      Serial.println(RED_CIRCLE_D);
    }
    else if(buf[0] == 'b'){
      Serial.println(BLUE_CIRCLE_D);
    }
    else if(buf[0] == 'c'){
      Serial.println(GREEN_CIRCLE_D);
    }
    else if(buf[0] == 'f'){
      Serial.println(RED_TRI_1_D);
    }
    else if(buf[0] == 'd'){
      Serial.println(BLUE_CROSS_1_D);
    }
    else if(buf[0] == 'i'){
      Serial.println(LIGHT_OFF_D);
    }
  }
}
void BT_lightandfire_rx(){
  while(Serial2.available()>0){
    char buf[100];
    int len = Serial2.readBytes(buf,100);
    if(buf[0] == 'e'){
      Serial.println(BLUE_CROSS_2_D);
    }
    else if(buf[0] == 'g'){
      Serial.println(RED_TRI_2_D);
    }
    else if(buf[0] == 'h'){
      Serial.println(RED_TRI_3_D);
    }
    else if(buf[0] == 'i'){
      // Serial.println(LIGHT_OFF_D);
    }
    else if(buf[0] == 'j'){
      Serial.println(FIRE_ON_D);
    }
  }
}
void BT_water_rx(){
  while(BTserial.available()>0){
    char buf[100];
    int len = BTserial.readBytes(buf,100);
    if(buf[0] == 'k'){
      Serial.println(WATER_ON_D);
    }
  }
}
void BT_car_rx(){
  while(Serial3.available()>0){
    char buf[20];
    int len = Serial3.readBytes(buf,20);
    if(buf[0] == 'l'){
      Serial.println(CAR_OVER_D);
    }
  }
}
void BT_main_tx(){
  while(Serial.available()>0){
    char buf[20];
    int len = Serial.readBytes(buf,20);
    if(buf[0] == 'A'){
      Serial1.write(RED_CIRCLE);
      Serial.println(RED_CIRCLE);
    }
    else if(buf[0] == 'B'){
      Serial1.write(BLUE_CIRCLE);
      Serial.println(BLUE_CIRCLE);
    }
    else if(buf[0] == 'C'){
      Serial1.write(GREEN_CIRCLE);
      Serial.println(GREEN_CIRCLE);
    }
    else if(buf[0] == 'F'){
      Serial1.write(RED_TRI_1);
      Serial.println(RED_TRI_1);
    }
    else if(buf[0] == 'D'){
      Serial1.write(BLUE_CROSS_1);
      Serial.println(BLUE_CROSS_1);
    }
    else if(buf[0] == 'E'){
      Serial2.write(BLUE_CROSS_2);
      Serial.println(BLUE_CROSS_2);
    }
    else if(buf[0] == 'G'){
      Serial2.write(RED_TRI_2);
      Serial.println(RED_TRI_2);
    }
    else if(buf[0] == 'H'){
      Serial2.write(RED_TRI_3);
      Serial.println(RED_TRI_3);
    }
    else if(buf[0] == '1'){
      Serial2.write(FIRE_ON);
      Serial.println(FIRE_ON);
    }
    else if(buf[0] == 'I'){
      Serial1.write(LIGHT_OFF);
      Serial2.write(LIGHT_OFF);
      Serial.println(LIGHT_OFF);
    }
    else if(buf[0] == '2'){
      BTserial.write(WATER_ON);
      Serial.println(WATER_ON);
    }
    else if(buf[0] == '3'){
      Serial3.write(CAR_FRONT);
    }
    else if(buf[0] == '4'){
      Serial3.write(CAR_BACK);
    }
    else if(buf[0] == '5'){
      Serial3.write(CAR_LEFT);
    }
    else if(buf[0] == '6'){
      Serial3.write(CAR_RIGHT);
    }
    else if(buf[0] == '7'){
      Serial3.write(CAR_OFF);
    }
    else if(buf[0] == '8'){
      Serial3.write(CAR_OVER);
      Serial.println(CAR_OVER);
    }
  }
}
