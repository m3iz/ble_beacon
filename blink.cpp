#include "blink.h"
#include <math.h>

int LED1 = 22;
int LED2 = 21;  //2

int dvalue = 0;
void BLINK_init() {
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void helloBlink(){
  for(int i=0;i<15;i++){
  digitalWrite(LED2, HIGH);
  delay(50);  //x^2 как вариант
  digitalWrite(LED2, LOW);
  delay(50);
  }
}

void BLINK_red() {
  if (inZone) {
    digitalWrite(LED2, HIGH);
    Serial.print("Middle value= ");
    Serial.print(mval);
  }

  if (mval >= 0 && mval <= 40) {
    dvalue = 50;
  } else if (mval > 40 && mval <= 50) {
    dvalue = 100;
  }
  else if (mval > 50 && mval <= 55) {
    dvalue = 150;
  } else if (mval > 55 && mval <= 60) {
    digitalWrite(LED1, HIGH);
    dvalue = 200;
  } else if (mval > 60 && mval <= 70) {
    digitalWrite(LED1, LOW);
    dvalue = 300;
  } else if (mval > 70 && mval <= 80) {
    digitalWrite(LED1, LOW);
    dvalue = 600;
  } else {
    digitalWrite(LED1, LOW);
    dvalue = 900;
  }

  delay(dvalue);  //x^2 как вариант
  digitalWrite(LED2, LOW);
  delay(dvalue);
}
