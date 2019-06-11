#include "GS_Hardware.h"

using namespace PINS;

void solenoidOn()
{
    digitalWrite(SOL_DIG_PIN1, HIGH);
    digitalWrite(SOL_DIG_PIN2, HIGH);
    digitalWrite(SOL_DIG_PIN3, HIGH);
}

void solenoidOff()
{
    digitalWrite(SOL_DIG_PIN1, LOW);
    digitalWrite(SOL_DIG_PIN2, LOW);
    digitalWrite(SOL_DIG_PIN3, LOW);

}

void actIn()
{
  digitalWrite(ACT_DIG_PIN1, HIGH);
  digitalWrite(ACT_DIG_PIN2, LOW);
}

void actOut()
{
    digitalWrite(ACT_DIG_PIN1, LOW);
  digitalWrite(ACT_DIG_PIN2, HIGH);

}


void checkCont(uint8_t cont[2])
{

    float solCont = (analogRead(SOL_ANALOG_IN))*(5.0/1023.0);
    float actCont = (analogRead(ACT_ANALOG_IN))*(5.0/1023.0);
    
    Serial.println(solCont, DEC);
    Serial.println(actCont, DEC);
    //Serial.println(ematchCont, DEC);
    if ((solCont > 0.9) && (solCont < 1.8))
    {
       cont[0] = 1;

    }
    else{
        cont[0] = 0;
    }
      if ((actCont > 1) && (actCont < 2))
    {
       cont[1] = 1;

    }
    else
    {
        cont[1] = 0;
    }
   
}

void pinsOff()
{
    
    digitalWrite(SOL_DIG_PIN1, LOW);
    digitalWrite(SOL_DIG_PIN2, LOW);
    digitalWrite(SOL_DIG_PIN3, LOW);
    digitalWrite(ACT_DIG_PIN1, LOW);
    digitalWrite(ACT_DIG_PIN2, LOW);
    

    
}

void actOff()
{
   digitalWrite(ACT_DIG_PIN1, LOW);
    digitalWrite(ACT_DIG_PIN2, LOW);
}

