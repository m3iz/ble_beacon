#include "blink.h"  
#include <math.h>

int LED1 = 21; 
int LED2 = 2;  

void BLINK_init(){
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void BLINK_red(){
  if(inZone){
    digitalWrite(LED2, HIGH);
  }
  
    delay(abs(slevel)*3); //x^2 как вариант
    digitalWrite(LED2, LOW);
    delay(abs(slevel)*3);
  
}




