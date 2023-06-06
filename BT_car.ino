// #include <SoftwareSerial.h>   // 引用程式庫
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
const int In1 = 4;
const int In2 = 5;
const int In3 = 6;      
const int In4 = 7;    
// 定義連接藍牙模組的序列埠
// SoftwareSerial BT(10,11); // 接收腳, 傳送腳
void mstop(){
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}
void mfront(){
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
}
void mback(){
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
}
void mleft(){
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}
void mright(){
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
}

void setup(){
  Serial.begin(9600);
  Serial1.begin(9600);
  // BT.begin(9600);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT); 
}
void loop(){
  while(Serial1.available()>0){
    char val[10];
    int len = Serial1.readBytes(val,10);
    // for(int i=0;i<len;i++){
    //   Serial.print(val[i]);
    // }
    if(val[0] == 3){
      mfront();
      Serial.println("front");
    }
    else if(val[0] == 4){
      mback();
      Serial.println("back");
    }
    else if(val[0] == 5){
      mleft();
      Serial.println("left");
    }
    else if(val[0] == 6){
      mright();
      Serial.println("right");
    }
    else if(val[0] == 7){
      mstop();
      Serial.println("stop");
    }
    else if(val[0] == 8){
      mstop();
      Serial.println("over");
      Serial1.write(CAR_OVER_D);
    }

  }
}