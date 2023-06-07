#include <FastLED.h>
#include <SoftwareSerial.h>
#define LED_PIN     12
#define NUM_LEDS    200
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
#define FIRE_ON_D "i"
#define WATER_ON_D "j"
#define CAR_OFF_D "k"
#define LIGHT_OFF_D "l"
CRGB leds[NUM_LEDS];
String data = "";  // 存儲接收到的資料

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
}

void loop() {
    char data[100];
    while (Serial1.available()>0) {  // 檢查藍牙模組是否有可用的訊息
      int len = Serial1.readBytes(data,100);  // 讀取藍牙訊息
      // 在序列監視器上顯示接收到的訊息
      for (int i = 0;i<len;i++){
        Serial.print(data[i]);
      }
      Serial.println("re");
      if (data[0] == 'A'){
        for (int i = 0; i < NUM_LEDS; i++){
          leds[i] = CRGB(100, 0, 0);
          FastLED.show();
          delay(40);
        }
        Serial1.write(RED_CIRCLE_D);
      }
      else if (data[0] == 'B'){ ;
        for (int i = 0; i < NUM_LEDS; i++){
          leds[i] = CRGB(0, 0, 100);
          FastLED.show();
          delay(40);
        }
        Serial1.write(BLUE_CIRCLE_D);
      }
      else if (data[0] == 'C'){ 
        for (int i = 0; i < NUM_LEDS; i++){
          leds[i] = CRGB(0, 100, 0);
          FastLED.show();
          delay(40);
        }
        Serial1.write(GREEN_CIRCLE_D);
      }
      else if (data[0] == 'I'){ ;
        digitalWrite(7, LOW);        
        digitalWrite(8, LOW);
        for (int i = NUM_LEDS; i >= 0; i--){
          leds[i] = CRGB(0, 0, 0);
          FastLED.show();
          delay(40);
        }
        Serial1.write(LIGHT_OFF_D);
      }
      else if (data[0] == 'F') {
          digitalWrite(7, HIGH);
          Serial1.write(RED_TRI_1_D);
      }
      else if (data[0] == 'D') {
          digitalWrite(8, HIGH);
          Serial1.write(BLUE_CROSS_1_D);
      }
      else{
        Serial.println("Error");
      }
  }
}
