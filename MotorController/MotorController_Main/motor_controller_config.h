#ifndef MOTOR_CONTROLLER_CONFIG_H_INCLUDED
#define MOTOR_CONTROLLER_CONFIG_H_INCLUDED

#include <Arduino.h>
#include <Encoder.h>
#include <stdio.h>


#define UserInput Serial
#define LOG Serial2

#define VENT_CONT_PIN A0
#define OX_VALVE_CONT_PIN A8
#define EMATCH_CONT_PIN A4
#define CHAMBER_TRANS_INPUT A2
#define OX_TRANS_INPUT A5
#define INJ_TEMP_INPUT A7
#define NOZ_TEMP_INPUT A3

/* set up a namespace for the LoRa Radio Items */
namespace LoRa
{
    extern const unsigned int CS_PIN;
    extern const unsigned int RST_PIN;
    extern const unsigned int INT_PIN;
    extern const double FREQ;


}

/* namespace for interactions form user */
namespace User
{
    extern const unsigned long BAUD;

}

namespace MC_Pins
{
    const unsigned int VENT_DIG_PIN1 = 28;
    const unsigned int VENT_DIG_PIN2 = 22;
    const unsigned int FLOAT_SWITCH_PIN = 46;
    const unsigned int OX_VALVE_DIG_PIN1 = 34;
    const unsigned int OX_VALVE_DIG_PIN2 = 38;
    const unsigned int EMATCH_PIN1 = 27;
    const unsigned int EMATCH_PIN2 = 33;
    const unsigned int EMATCH_PIN3 = 43;
    
    const unsigned int CH_A = 10;
    const unsigned int CH_B = 11;


}





#endif // MOTOR_CONTROLLER_CONIFG_H_INCLUDED
