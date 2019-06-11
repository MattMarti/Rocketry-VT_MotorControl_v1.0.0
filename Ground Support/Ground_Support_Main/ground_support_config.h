#ifndef GROUND_SUPPORT_CONFIG_H_INCLUDED
#define GROUND_SUPPORT_CONFIG_H_INCLUDED

#define SOL_ANALOG_IN A0
#define ACT_ANALOG_IN A1

#include <Arduino.h>

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

namespace PINS
{
    const unsigned int SOL_DIG_PIN1 = 49;
    const unsigned int SOL_DIG_PIN2 = 45;
    const unsigned int SOL_DIG_PIN3 = 47;
    //also uses a 5V input, and ground

    const unsigned ACT_DIG_PIN1 = 22;
    const unsigned ACT_DIG_PIN2 = 23;

}

#endif // GROUND_SUPPORT_CONFIG_H_INCLUDED
