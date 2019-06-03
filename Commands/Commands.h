#ifndef Commands_H_INCLUDED
#define Commands_H_INCLUDED

#include <Arduino.h> //allows for usage of uint8_t and so forth with arduino type def

/*
* this library is designed to define all commands that the host can send to
* the MC and GS. These commands are given over Serial user input
*
*/


/*name space to represent what serial command corresponds to what command
* will be sent to other systems, or back to user.
*/
namespace Commands
{
    const uint8_t SHOW_COMMANDS = 0x03; //print commands to Serial for user to see, HC
     const uint8_t UNLOCK1 = 0x04; //unlock 1 for transmitting, HC
     const uint8_t UNLOCK2 = 0x05; //unlock 2 for transmitting, HC
     const uint8_t UNLOCK3 = 0x06; //unlock 3, if all unlocked allow for certian commands to be sent, HC
    const uint8_t PING_STATE = 0x07; //PING STATE, ALL
     const uint8_t FILL = 0x08; //ALL
     const uint8_t DISCONNECT_FILL = 0x09; //GS
     const uint8_t LAUNCH = 0x10; //HC, MC

     //specific meaning must have been sent FROM
     //0xAC = host specific
     //0xCC = Motor controller specific
     //0xF5 = Ground support specific
     //0xDD = could be any





     const uint8_t PING_STATE_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x07, 0xAB, 0xAD};
     const uint8_t FILL_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x08, 0xAC, 0xAD};
    const uint8_t DISCONNECT_FILL_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x09, 0xDD, 0xDC};
    const uint8_t LAUNCH_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x10, 0xDE, 0xFE};

    const uint8_t MC_FILL_STATE[7] = {0xAA, 0x14, 0x01, 0xCC, 0x11, 0xAF, 0x71}; //Reprensents motor fill state
    const uint8_t MC_IDLE_STATE[7] = {0xAA, 0x14, 0x01, 0xCC, 0x12, 0xAF, 0x71}; //Represnts MOTOR idle state, not ready to do anythin
    const uint8_t MC_READY_STATE[7] = {0xAA, 0x14, 0x01, 0xCC, 0x13, 0xAF, 0x71}; //Represnets motor ready to start doing things
    const uint8_t MC_LAUNCH_STATE[7] = {0xAA, 0x14, 0x01, 0xCC, 0x14, 0xAF, 0x71}; //represnets motor launched


        const uint8_t GS_FILL_STATE[7] = {0xAA, 0x14, 0x01, 0xF5, 0x15, 0xBF, 0x91}; //Reprensents GROUND SUPPORT fill state
    const uint8_t GS_IDLE_STATE[7] = {0xAA, 0x14, 0x01, 0xF5, 0x16, 0xDF, 0x75}; //Represnts GROUND SUPPORT idle state, not ready to do anythin
    const uint8_t GS_READY_STATE[7] = {0xAA, 0x14, 0x01, 0xF5, 0x17, 0xEF, 0x7E}; //Represnets GORUND SUPPORT ready to start doing things
    const uint8_t GS_LAUNCH_STATE[7] = {0xAA, 0x14, 0x01, 0xF5, 0x18, 0xA1, 0xE1}; //represnets GROUND launched




}



#endif // HOST_SERIAL_INTERPRET_H_INCLUDED
