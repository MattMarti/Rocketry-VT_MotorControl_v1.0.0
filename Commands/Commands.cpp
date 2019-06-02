#include "Commands.h"


/*Declare Commands */

    const uint8_t Commands :: SHOW_COMMANDS = 0x03; //print commands to Serial for user to see, HC
     const uint8_t Commands :: UNLOCK = 0x04; //unlock 1 for transmitting, HC
     const uint8_t Commands :: UNLOCK2 = 0x05; //unlock 2 for transmitting, HC
     const uint8_t Commands :: UNLOCK3 = 0x06; //unlock 3, if all unlocked allow for certian commands to be sent, HC
    const uint8_t Commands :: PING_STATE = 0x07; //PING STATE, ALL
     const uint8_t Commands :: FILL = 0x08; //ALL
     const uint8_t Commands :: DISCONNECT_FILL = 0x09; //GS
     const uint8_t Commands :: LAUNCH = 0x10; //HC, MC

     const uint8_t Commands :: PING_STATE_PACKET[7] = {0xAA, 0x14, 0xHC, 0x07, 0xAB, 0xAA};
     const uint8_t Commands :: FILL_PACKET[7] = {0xAA, 0x14, 0xHC, 0x08, 0xAC, 0xAA};
    const uint8_t Commands :: DISCONNECT_FILL_PACKET[7] = {0xAA, 0x14, 0xHC, 0x09, 0xDD, 0xDC};
    const uint8_t Commands :: LAUNCH_PACKET[7] = {0xAA, 0x14, 0xHC, 0x10, 0xFU, 0xFM};
