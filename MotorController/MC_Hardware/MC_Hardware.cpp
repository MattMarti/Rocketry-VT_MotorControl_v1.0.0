#include "MC_Hardware.h"

using namespace MC_Pins;
void checkCont(uint8_t cont[3])
{

    float ventCont = (analogRead(VENT_CONT_PIN))*(5.0/1023.0);
    float oxCont = (analogRead(OX_VALVE_CONT_PIN))*(5.0/1023.0);
    float ematchCont = (analogRead(EMATCH_CONT_PIN))*(5.0/1023.0);
    Serial.println(ventCont, DEC);
    Serial.println(oxCont, DEC);
    Serial.println(ematchCont, DEC);
    if ((ventCont > 1) && (ventCont < 2))
    {
       cont[0] = 1;

    }
    else{
        cont[0] = 0;
    }
      if ((oxCont > 1) && (oxCont < 2))
    {
       cont[1] = 1;

    }
    else
    {
        cont[1] = 0;
    }
    if ((ematchCont > 0.45) && (ematchCont < 1))
    {
        cont[2] = 1;
    }
    else{
        cont[2] = 0;
    }




}

bool tankFull()
{
    if (digitalRead(FLOAT_SWITCH_PIN) == 0)
    {

        return true;
    }
    else{
        return false;
    }
}

    void openVentFull()
    {
        digitalWrite(VENT_DIG_PIN1, LOW);
        digitalWrite(VENT_DIG_PIN2, HIGH);
    }

void closeVent()
{
    digitalWrite(VENT_DIG_PIN1, HIGH);
        digitalWrite(VENT_DIG_PIN2, LOW);

}

void crackVent()
{
    digitalWrite(VENT_DIG_PIN1, LOW);
        digitalWrite(VENT_DIG_PIN2, HIGH);
        delay(100);
        digitalWrite(VENT_DIG_PIN1, LOW);
        digitalWrite(VENT_DIG_PIN2, LOW);

}

void openOxValve()
{
    digitalWrite(OX_VALVE_DIG_PIN1, HIGH);
        digitalWrite(OX_VALVE_DIG_PIN2, LOW);

}

void closeOxValve()
{
     digitalWrite(OX_VALVE_DIG_PIN1, LOW);
        digitalWrite(OX_VALVE_DIG_PIN2, HIGH);

}

void pinsOff()
{
    digitalWrite(EMATCH_PIN1, LOW);
    digitalWrite(EMATCH_PIN2, LOW);
    digitalWrite(EMATCH_PIN3, LOW);
    digitalWrite(VENT_DIG_PIN1, LOW);
        digitalWrite(VENT_DIG_PIN2, LOW);
        digitalWrite(OX_VALVE_DIG_PIN1, LOW);
        digitalWrite(OX_VALVE_DIG_PIN2, LOW);
}

void fireEmatch()
{
    digitalWrite(EMATCH_PIN2, HIGH);
    delay(2000);
    digitalWrite(EMATCH_PIN1, HIGH);
    delay(2000);
    digitalWrite(EMATCH_PIN3, HIGH);
    delay(2000);


}

uint8_t readChamberPressure()
{
    float chamberPressure = (((analogRead(CHAMB_PRES_INPUT))*(5.0/1023.0))*17.2369)-8.61845;
    return (uint8_t) chamberPressure;
}
