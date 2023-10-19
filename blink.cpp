#include "blink.h"
#include <math.h>

int LED1 = 22;
int LED2 = 21;  //21
bool led=false;
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
    if(!led){
      digitalWrite(LED2, HIGH);
    }else{
      digitalWrite(LED1, HIGH);
    }
  }

  if(mval<65)led=true;
  else led = false;
  delay(150);  //x^2 как вариант
  digitalWrite(LED2, LOW);
  digitalWrite(LED1, LOW);
  delay(150);
}
