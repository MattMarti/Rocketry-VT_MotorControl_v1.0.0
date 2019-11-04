#ifndef MC_HARDWARE_H_INCLUDED
#define MC_HARDWARE_H_INCLUDED


#include "motor_controller_config.h"

void checkCont(uint8_t cont[3]);

bool tankFull();

void openVentFull();

void closeVent();

void crackVent();

void openOxValve();

void closeOxValve();

void pinsOff();

void fireEmatch();
#endif // MC_HARDWARE_H_INCLUDED

uint8_t readChamberPressure();

uint8_t readOxPressure();

uint8_t readInjTemp();

uint8_t readNozTemp();

//uint8_t readVentPos();

//uint8_t readOxPos();

