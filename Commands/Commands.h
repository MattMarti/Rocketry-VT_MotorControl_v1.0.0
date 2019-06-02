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
    extern const uint8_t SHOW_COMMANDS; //print commands to Serial for user to see, HC
        extern const uint8_t UNLOCK1; //unlock 1 for transmitting, HC
    extern const uint8_t UNLOCK2; //unlock 2 for transmitting, HC
    extern const uint8_t UNLOCK3; //unlock 3, if all unlocked allow for certian commands to be sent, HC
    extern const uint8_t PING_STATE; //PING STATE, ALL
    extern const uint8_t FILL; //ALL
    extern const uint8_t DISCONNECT_FILL; //GS
    extern const uint8_t LAUNCH; //HC, MC

    extern const uint8_t PING_STATE_PACKET[7];
    extern const uint8_t FILL_PACKET[7];
    extern const uint8_t DISCONNECT_FILL_PACKET[7];
    extern const uint8_t LAUNCH_PACKET[7];



}



#endif // HOST_SERIAL_INTERPRET_H_INCLUDED
