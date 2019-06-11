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
    delay(3000);
    digitalWrite(EMATCH_PIN1, HIGH);
    delay(3000);
    digitalWrite(EMATCH_PIN3, HIGH);
    //delay(2000);


}

uint8_t readChamberPressure()
{
    float chamberPressure = (((analogRead(CHAMBER_TRANS_INPUT))*(5.0/1023.0))*17.2369)-8.61845;
    Serial.println(chamberPressure, DEC);
    return (uint8_t) chamberPressure;
}


uint8_t readOxPressure()
{
    float oxPressure = (((analogRead(OX_TRANS_INPUT))*(5.0/1023.0))*17.2369)-8.61845;
    Serial.println(oxPressure, DEC);
    return (uint8_t) oxPressure;
}


uint8_t readInjTemp()
{
    float inj_mVolt = (analogRead(INJ_TEMP_INPUT))*(5000.0/1023.0);

    //account for the voltage divider
    float V = inj_mVolt/(69.0/79.0);
    Serial.println("inj mV:");
    Serial.println(V, DEC);
    //From AOE elctronics
    //float C0 = 0;
    //float C1 = 2.5084*(10.0);
    //float C2 = 7.86011*(0.1);
    //float C3 = -2.503;
    //float C4 = 8.3153*(0.1);
    //float C5 = -1.228*(0.1);
   // float C6 = 9.804036*(0.001);
    //float C7 = -4.413030*(0.0001);
    //float C8 = 1.057734*(0.00001);
    //float C9 = -1.052755*(0.0000001);
   // double temp = C0+(C1*V)+(C2*(pow(V, 2)))+(C3*(pow(V,3)))+(C4*pow(V,4));//+(C5*pow(V,5))+(C6*pow(V,6));//+(C7*pow(V,7))+(C8*pow(V,8))+(C9*pow(V,9));
    //temp = (temp*(9.0/5.0))+32.0;
    double temp = (1.2273*V)-48.2; //F
    Serial.println("temp inj:");
    Serial.println(temp, DEC);
    return (uint8_t) (temp/10.0);




}

uint8_t readNozTemp()
{
    float noz_mVolt = (analogRead(NOZ_TEMP_INPUT))*(5000.0/1023.0);

    //account for the voltage divider
    float V = noz_mVolt/(69.0/79.0);
    Serial.println("noz mV:");
    Serial.println(V, DEC);
   // From AOE elctronics
    float C0 = 0;
    float C1 = 2.5084*(10.0);
    float C2 = 7.86011*(0.1);
    float C3 = -2.503;
    float C4 = 8.3153*(0.1);
    float C5 = -1.228*(0.1);
    float C6 = 9.804036*(0.001);
    float C7 = -4.413030*(0.0001);
    float C8 = 1.057734*(0.00001);
    float C9 = -1.052755*(0.0000001);
    double temp = C0+(C1*V)+(C2*(pow(V, 2)))+(C3*(pow(V,3)))+(C4*pow(V,4))+(C5*pow(V,5))+(C6*pow(V,6))+(C7*pow(V,7))+(C8*pow(V,8))+(C9*pow(V,9));
    temp = (temp*(9.0/5.0))+32.0;
   // double temp = (1.2273*V)-48.2; //F
    Serial.println("temp noz:");
    Serial.println(temp, DEC);
    return (uint8_t) (temp/10.0);




}

//uint8_t readVentPos()
//{
//  Encoder encoder(CH_A, CH_B);  // Definition of incrementation encoder
//  uint8_t pos = encoder.read();
//
//
//  double raw_degree = pos * (360.0 / 4096.0)*(4.0/3.0);
//  if (raw_degree < 0)
//  {
//    return 360.0 + raw_degree;
//  }
//  return (uint8_t) raw_degree;
//}

//uint8_t readOxPos()
//{
//  
//}

