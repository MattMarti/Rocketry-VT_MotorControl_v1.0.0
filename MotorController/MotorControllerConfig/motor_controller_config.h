#ifndef MOTOR_CONTROLLER_CONFIG_H_INCLUDED
#define MOTOR_CONTROLLER_CONFIG_H_INCLUDED



#define UserInput Serial



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





#endif // MOTOR_CONTROLLER_CONIFG_H_INCLUDED
