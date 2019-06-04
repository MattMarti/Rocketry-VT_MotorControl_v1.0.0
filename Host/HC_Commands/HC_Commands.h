#ifndef HC_Commands_H_INCLUDED
#define HC_Commands_H_INCLUDED

#include <Arduino.h> //allows for usage of uint8_t and so forth with arduino type def

/*
* this library is designed to define all commands that the host can send to
* the MC and GS. These commands are given over Serial user input
*
*/


/*name space to represent what serial command corresponds to what command
* will be sent to other systems, or back to user.
*/
namespace HC_Commands
{
    const uint8_t SHOW_COMMANDS = 0x03; //print commands to Serial for user to see, HC
     const uint8_t UNLOCK1 = 0x04; //unlock 1 for transmitting, HC
     const uint8_t UNLOCK2 = 0x05; //unlock 2 for transmitting, HC
     const uint8_t UNLOCK3 = 0x06; //unlock 3, if all unlocked allow for certian commands to be sent, HC
    const uint8_t PING_STATE_All = 0x07; //PING STATE, ALL

     const uint8_t FILL = 0x08; //ALL
     const uint8_t DISCONNECT_FILL = 0x09; //GS
     const uint8_t LAUNCH = 0x10; //HC, MC
     const uint8_t PING_STATE_MC = 0x19; //PING STATE, motor controller
     const uint8_t PING_STATE_GS = 0x20; //PING STATE, ground support

     const uint8_t PING_STATE_HC = 0x21; //PING STATE, host


     const uint8_t MAKE_READY = 0x23; //makes everything ready to start transmitting n fill
     const uint8_t PAUSE = 0x24; //CLOSE all valves  and wait
     const uint8_t ABORT = 0x25;
     const uint8_t SET_VENT_PRESET_PRES = 0x26; //vent to a preset pressure
     const uint8_t VENT_MANUAL = 0x27; //just open the vent valve
     const uint8_t PING_TANK_PRESSURE = 0x28; //PING ox tank pressure
     const uint8_t RECC_LAUNCH_PROCEDURE = 0x29; //print reccomended command sequence for launch
      const uint8_t PING_CHAMBER_PRESSURE = 0x30; //ping pressure in comb chamber
      const uint8_t PING_INJ_TEMP = 0x31; //ping the temeperature of the chamber by the injector
       const uint8_t PING_NOZ_TEMP = 0x32; //ping temp of chamber casing by nozzle
       const uint8_t PING_ALL_DATA = 0x33; //ping for all ze data
       const uint8_t ENABLE_AUTO_VENT = 0x34; //allow for the system to vent automatically to preset pressure
       const uint8_t DISABLE_AUTO_VENT = 0x35;
       const uint8_t ENABLE_AUTO_PAUSE = 0x36;
       const uint8_t DISABLE_AUTO_PAUSE = 0x37;
       const uint8_t PING_ALL_SETTINGS = 0x38;
       const uint8_t PING_MC_SETTINGS = 0x39;
       const uint8_t PING_GS_SETTINGS = 0x40;
       const uint8_t PING_VENT_POS = 0x41;
       const uint8_t CLOSE_VENT = 0x42;
       const uint8_t PING_PRESET_PRES = 0X43;
       const uint8_t RESUME = 0X97;
       const uint8_t ENABLE_WARNINGS = 0x98;
       const uint8_t DISABLE_WARNINGS = 0x99;
       const uint8_t DECLARE_TANK_FULL = 0x96;
       const uint8_t DECLARE_TANK_NOT_FULL = 0x93;
       const uint8_t DISABLE_FLOAT_SWITCH = 0x95;
       const uint8_t ENABLE_FLOAT_SWITCH = 0x94;




     //specific meaning must have been sent FROM
     //0xAC = host specific
     //0xCC = Motor controller specific
     //0xF5 = Ground support specific
     //0xDD = could be any



/*
*stuff to send to MC and GS
*/

     const uint8_t PING_STATE_PACKET_MC[7] = {0xAA, 0x14, 0x01, 0xAC, 0x019, 0xAB, 0xAD};
     const uint8_t PING_STATE_PACKET_GS[7] = {0xAA, 0x14, 0x01, 0xAC, 0x20, 0xAB, 0xAD};
     const uint8_t FILL_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x08, 0xAC, 0xAD};
    const uint8_t DISCONNECT_FILL_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x09, 0xDD, 0xDC};
    const uint8_t LAUNCH_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x10, 0xDE, 0xFE};
     const uint8_t MAKE_READY_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x23, 0xDE, 0xCE}; //makes everything ready to start transmitting n fill
     const uint8_t PAUSE_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x24, 0x1E, 0x9E}; //CLOSE all valves  and wait, doesn't reset predicates
     const uint8_t ABORT_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x25, 0x1D, 0x5E};
     const uint8_t SET_PRESET_VENT_PRES_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x26, 0xD1, 0x2F}; //set pressure to auto vent to
     const uint8_t VENT_MANUAL_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x27, 0x1E, 0x3E}; //just open the vent valve
     const uint8_t PING_TANK_PRESSURE_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x28, 0xD3, 0x4E}; //PING pressure
       const uint8_t PING_CHAMBER_PRESSURE_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x30, 0x9A, 0xA8};//ping pressure in comb chamber
      const uint8_t PING_INJ_TEMP_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x31, 0xD2, 0xEE}; //ping the temeperature of the chamber by the injector
       const uint8_t PING_NOZ_TEMP_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x32, 0xC8, 0x9E};//ping temp of chamber casing by nozzle
       const uint8_t PING_ALL_DATA_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x33, 0x9F, 0x7E}; //ping for all ze data
       const uint8_t ENABLE_AUTO_VENT_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x34, 0xA3, 0xBE}; //allow for the system to vent automatically to preset pressure
       const uint8_t DISABLE_AUTO_VENT_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x35, 0xDB, 0xDE};
       const uint8_t ENABLE_AUTO_PAUSE_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x36, 0xAF, 0xFA};
       const uint8_t DISABLE_AUTO_PAUSE_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x37, 0xFC, 0xCF};
       const uint8_t PING_ALL_SETTINGS_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x38, 0xE3, 0xEE};
       const uint8_t PING_MC_SETTINGS_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x39, 0xE9, 0x9E};
      //`3`32`11```````1` const uint8_t PING_GS_SETTINGS_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x40, 0xF7, 0xFB};
       const uint8_t PING_VENT_POS_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x41, 0xC7, 0xEB};
       const uint8_t CLOSE_VENT_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x42, 0xF2, 0xCB};
       const uint8_t SET_PRESET_PRES_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x43, 0xF2, 0xCB};
       const uint8_t ENABLE_WARNINGS_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x98, 0xF6, 0xC7};
       const uint8_t DISABLE_WARNINGS_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x99, 0xD6, 0xE7};
       const uint8_t RESUME_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x97, 0xD7, 0xE6};
         const uint8_t DECLARE_TANK_FULL_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x96, 0x1A, 0x2A};
       const uint8_t DECLARE_TANK_NOT_FULL_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x93, 0x4A, 0x2A};
       const uint8_t DISABLE_FLOAT_SWITCH_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x95, 0x1A, 0x2D};
       const uint8_t ENABLE_FLOAT_SWITCH_PACKET[7] = {0xAA, 0x14, 0x01, 0xAC, 0x94, 0x9A, 0x9A};


    /*
*stuff to send to HC
*/

//MC STUFF
    const uint8_t MC_FILL_STATE[7] = {0xAA, 0x14, 0x01, 0xCC, 0x11, 0xAF, 0xAE}; //Reprensents motor filling state
    const uint8_t MC_IDLE_STATE[7] = {0xAA, 0x14, 0x01, 0xCC, 0x12, 0xBF, 0xBB}; //Represnts MOTOR idle state, not ready to do anythin, valves closed
    const uint8_t MC_READY_STATE[7] = {0xAA, 0x14, 0x01, 0xCC, 0x13, 0xBD, 0xD4}; //Represnets motor ready to start doing things
    const uint8_t MC_LAUNCH_STATE[7] = {0xAA, 0x14, 0x01, 0xCC, 0x14, 0xA4, 0x7A}; //represnets motor launched
    const uint8_t MC_TANK_FULL_STATE[7] = {0xAA, 0x14, 0x01, 0xCC, 0x22, 0xCF, 0xE1}; //Reprensents full tank
    const uint8_t MC_ABORT_STATE[7] = {0xAA, 0x14, 0x01, 0xCC, 0x44, 0xDF, 0xE7};
    const uint8_t MC_VENT_POS_PACKET[8] = {0xAA, 0x14, 0x02, 0xCC, 0x45, 0x00, 0xCF, 0xA7};  //, the second data param = posistion, starts as 0, will be written with new local copy
     const uint8_t PRESET_PRES_PACKET[8] = {0xAA, 0x14, 0x02, 0xCC, 0x46, 0x62, 0xAF, 0xEE};  //non constant, the second data param = posistion, starts as 62 bar (900 psi)
     const uint8_t TANK_PRES_PACKET[8] = {0xAA, 0x14, 0x02, 0xCC, 0x47, 0x02, 0xDD, 0xE1};  //non constant, SECOND DATA PARAM = TANK PRES IN BAR ^
      const uint8_t CHAMB_PRES_PACKET[8] = {0xAA, 0x14, 0x02, 0xCC, 0x48, 0x00, 0xDF, 0xE7};  //non constant, the second data param = CHAMB PRES BAR ^
    const  uint8_t INJ_TEMP_PACKET[8] = {0xAA, 0x14, 0x02, 0xCC, 0x49, 0x07, 0xDF, 0xE7};  //non constant, the second data param = TEMP IN F/10, STARTS AS 2 ^
     const uint8_t NOZ_TEMP_PACKET[8] = {0xAA, 0x14, 0x02, 0xCC, 0x50, 0x07, 0xD3, 0xB7};  //non constant, the second data param = TEMP IN F/10, STARTS AS 2 ^
     const uint8_t ALL_DATA_PACKET[11] = {0xAA, 0x14, 0x04, 0xCC, 0x51, 0x02, 0x00, 0x07, 0x07, 0xD9, 0x9D};  //non constant, ALL DATA, IN TANK PRES, CHAMB PRES, INJ TEMP, NOZ TEMP ORDER
      const uint8_t AUTO_VENT_ENABLED[7] = {0xAA, 0x14, 0x01, 0xCC, 0x52, 0xC1, 0xB7};
      const uint8_t AUTO_VENT_DISABLED[7] = {0xAA, 0x14, 0x01, 0xCC, 0x53, 0xC5, 0xB2};
      const uint8_t VENTING_ON[7] = {0xAA, 0x14, 0x01, 0xCC, 0x54, 0xC7, 0xB3};
      const uint8_t VENTING_OFF[7] = {0xAA, 0x14, 0x01, 0xCC, 0x55, 0xC9, 0xC3};
      const uint8_t AUTO_PAUSE_ENABLED[7] = {0xAA, 0x14, 0x01, 0xCC, 0x56, 0xC2, 0xD3};
      const uint8_t AUTO_PAUSE_DISABLED[7] = {0xAA, 0x14, 0x01, 0xCC, 0x57, 0xB2, 0xD3};
      const uint8_t MC_AUTO_PAUSED[7] = {0xAA, 0x14, 0x01, 0xCC, 0x58, 0xC2, 0xD3};
      const uint8_t WARNINGS_ENABLED[7] = {0xAA, 0x14, 0x01, 0xCC, 0x59, 0xC5, 0xD};
      const uint8_t WARNINGS_DISABLED[7] = {0xAA, 0x14, 0x01, 0xCC, 0x60, 0xB5, 0xD5};
      const uint8_t MC_MAN_PAUSED[7] = {0xAA, 0x14, 0x01, 0xCC, 0x61, 0xE5, 0xB5};
      const uint8_t MC_OVER_TEMP_WARNING[7] = {0xAA, 0x14, 0x01, 0xCC, 0x70, 0xE1, 0xC5};
      const uint8_t MC_OVER_PRES_WARNING[7] = {0xAA, 0x14, 0x01, 0xCC, 0x71, 0xD1, 0xD5};
      const uint8_t MC_LOW_PRES_DROP_WARNING[7] = {0xAA, 0x14, 0x01, 0xCC, 0x72, 0xB1, 0xB5};






/*GS stuff*/


        const uint8_t GS_FILL_STATE[7] = {0xAA, 0x14, 0x01, 0xF5, 0x15, 0xBF, 0xE1}; //Reprensents GROUND SUPPORT fill state
    const uint8_t GS_IDLE_STATE[7] = {0xAA, 0x14, 0x01, 0xF5, 0x16, 0xDF, 0xA5}; //Represnts GROUND SUPPORT idle state, not ready to do anythin
    const uint8_t GS_READY_STATE[7] = {0xAA, 0x14, 0x01, 0xF5, 0x17, 0xEF, 0x7E}; //Represnets GORUND SUPPORT ready to start doing things
    const uint8_t GS_LAUNCH_STATE[7] = {0xAA, 0x14, 0x01, 0xF5, 0x18, 0xA1, 0xD3}; //represnets GROUND launched
    const uint8_t GS_ABORT_STATE[7] = {0xAA, 0x14, 0x01, 0xF5, 0x62, 0xA3, 0xD4};
    const uint8_t GS_AUTO_PAUSED[7] = {0xAA, 0x14, 0x01, 0xF5, 0x63, 0xA2, 0xD3};
    const uint8_t GS_DISCONNECTED[7] = {0xAA, 0x14, 0x01, 0xF5, 0x64, 0xA9, 0xD1};






}

void printCommands();

void explain();


#endif // HOST_SERIAL_INTERPRET_H_INCLUDED
