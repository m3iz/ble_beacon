#ifndef __BLINK_H__
#define __BLINK_H__ 1
#include <Arduino.h> 

extern bool inZone;
extern int slevel;
void BLINK_init(void);  
void BLINK_green(void);  
void BLINK_yellow(void);  
void BLINK_red(void);  
#endif