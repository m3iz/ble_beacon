#include "blink.h"
#include <math.h>

int LED1 = 21;
int LED2 = 2;  //2

int dvalue = 0;
void BLINK_init() {
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void BLINK_red() {
  if (inZone) {
    digitalWrite(LED2, HIGH);
  }

  if (slevel >= 0 && slevel <= 20) {
    dvalue = 50;
  } else if (slevel > 20 && slevel <= 30) {
    dvalue = 50;
  } else if (slevel > 30 && slevel <= 40) {
    dvalue = 50;
  } else if (slevel > 40 && slevel <= 50) {
    dvalue = 100;
  }
  else if (slevel > 50 && slevel <= 55) {
    dvalue = 200;
  } else if (slevel > 55 && slevel <= 60) {
    dvalue = 300;
  } else if (slevel > 60 && slevel <= 70) {
    dvalue = 600;
  } else if (slevel > 70 && slevel <= 80) {
    dvalue = 800;
  } else {
    dvalue = 900;
  }

  delay(dvalue);  //x^2 как вариант
  digitalWrite(LED2, LOW);
  delay(dvalue);
}
