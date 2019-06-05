#ifndef GROUND_SUPPORT_CONFIG_H_INCLUDED
#define GROUND_SUPPORT_CONFIG_H_INCLUDED

#define ANALOG_IN A0;
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

namespace Solenoid_Pins
{
    extern const unsigned int DIGITAL_PIN1;
    extern const unsigned int DIGITAL_PIN2;
    extern const unsigned int DIGITAL_PIN3;
    //also uses a 5V input, and ground

}

#endif // GROUND_SUPPORT_CONFIG_H_INCLUDED
