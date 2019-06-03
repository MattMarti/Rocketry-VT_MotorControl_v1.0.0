#ifndef GROUND_SUPPORT_CONFIG_H_INCLUDED
#define GROUND_SUPPORT_CONFIG_H_INCLUDED

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


#endif // GROUND_SUPPORT_CONFIG_H_INCLUDED
