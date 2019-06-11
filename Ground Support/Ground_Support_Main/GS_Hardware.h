#ifndef GS_HARDWARE_H_INCLUDED
#define GS_HARDWARE_H_INCLUDED

#include "ground_support_config.h"

void solenoidOn();

void solenoidOff();

void actOut();

void actIn();

void checkCont(uint8_t cont[2]);

void pinsOff();

 void actOff();


#endif // GS_HARDWARE_H_INCLUDED
