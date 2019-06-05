#include "HC_Commands.h"



/*Declare Commands */

void printCommands()
{
    Serial.println("\nThe following commands are available: ");
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
    Serial.println("Make all Motor Controller and Ground Support Ready: 0x00 0x23 0xFF");
    Serial.println("Pause the sequence (close all valves): 0x00 0x24 0xFF");
    Serial.println("Abort (dump nitrous, close valves, disconnect fill): 0x00 0x25 0xFF");
    Serial.println("Set the Vent to Pressure: 0x00 0x26 0xFF");
    Serial.println("Manually Command Venting: 0x00 0x27 0xFF");
    Serial.println("Ping Oxidizer Tank Pressure: 0x00 0x28 0xFF");
    Serial.println("Print an Explanation of Launch Procedure: 0x00 0x29 0xFF");
    Serial.println("Ping Combustion Chamber Pressure: 0x00 0x30 0xFF");
    Serial.println("Ping Casing Temp at Injector: 0x00 0x31 0xFF");
    Serial.println("Ping Casing Temp at Nozzle: 0x00 0x32 0xFF");
    Serial.println("Ping All Motor Data: 0x00 0x33 0xFF");
    Serial.println("Enable Auto-Venting to Control Ox Tank Pressure: 0x00 0x34 0xFF");
    Serial.println("Disable Auto-Venting to Control Ox Tank Pressure: 0x00 0x35 0xFF");
    Serial.println("Enable Auto-Pause for non-nominal conditions: 0x00 0x36 0xFF");
    Serial.println("Disable Auto-Pause for non-nominal conditions: 0x00 0x37 0xFF");
    Serial.println("Ping All System Settings: 0x00 0x38 0xFF");
    Serial.println("Ping Motor Controller Settings: 0x00 0x39 0xFF");
    Serial.println("Ping Ground Support Settings: 0x00 0x40 0xFF");
    Serial.println("Ping Vent Valve Position: 0x00 0x41 0xFF");
    Serial.println("Close Vent Valve: 0x00 0x42 0xFF");
    Serial.println("Ping the Vent Stop Pressure: 0x00 0x43 0xFF");
    Serial.println("Resume (if was paused): 0x00 0x97 0xFF");
    Serial.println("Enable Warnings for non-nominal conditions: 0x00 0x98 0xFF");
    Serial.println("Disable Warnings for non-nominal conditions: 0x00 0x98 0xFF");
    Serial.println("Declare tank not full (not reccomended, will override auto fill stops and allow to refill): 0x00 0x93 0xFF");
    Serial.println("Declare tank full: 0x00 0x96 0xFF");
    Serial.println("Disable float switch (not reccomended, will override auto fill stops): 0x00 0x95 0xFF");
    Serial.println("Enable float switch (enabled by default): 0x00 0x94 0xFF");
    Serial.println("Check Continuity on MC and GS Systems");




}










void explain()
{
    Serial.println("Unlock unlocks 1-3 to start transmitting data.");
    Serial.println("Send Make ready command for motor controller and ground support to be able to fill");
    Serial.println("Send fill command to start filling with nitrous oxide");
    Serial.println("Send disconnect fill line command to disconnect fill line");
    Serial.println("Send launch command to launch. All above commands must have been sent first");

}
