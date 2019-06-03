#include "Commands.h"



/*Declare Commands */

void printCommands()
{
    Serial.println("The following commands are available: ");
    Serial.println("Show Commands: 0x00 0x03 0xFF");
    Serial.println("Unlock1: 0x00 0x04 0xFF");
    Serial.println("Unlock2: 0x00 0x05 0xFF");
    Serial.println("Unlock3: 0x00 0x06 0xFF");
    Serial.println("Ping the States of All Systems: 0x00 0x07 0xFF");
    Serial.println("Start Tank Filling: 0x00 0x08 0xFF");
    Serial.println("Disconnect Fill Line: 0x00 0x09 0xFF");
    Serial.println("Launch Rocket: 0x00 0x10 0xFF");
    Serial.println("Ping State of Motor Controller: 0x00 0x19 0xFF");
    Serial.println("Ping State of Ground System: 0x00 0x20 0xFF");
    Serial.println("Ping State of Host Computer: 0x00 0x21 0xFF");



}
