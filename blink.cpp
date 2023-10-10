#include "blink.h"  
int LED1 = 21; 
int LED2 = 22;  
void BLINK_init(){
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}
void BLINK_green(){
    digitalWrite(LED1, HIGH);
    delay(1000);
    digitalWrite(LED1, LOW);
    delay(1000);
}
void BLINK_yellow(){
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    delay(300);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    delay(300);
}
void BLINK_red(){
    digitalWrite(LED2, HIGH);
    delay(50);
    digitalWrite(LED2, LOW);
    delay(50);  
}
