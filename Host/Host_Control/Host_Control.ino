#include <Host_Config.h>
#include <SPI.h>
#include <RH_RF95.h>
  using namespace LoRa;
  using namespace User;
RH_RF95 rf95(CS_PIN, INT_PIN);

void setup() {
  // put your setup code here, to run once:
  UserInput.begin(BAUD); //begin comms for user inputting (Serial)
  
  Serial.println("2019 Rocketry at VT Launch Control System");
 
      pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, HIGH);
    delay(100);
    digitalWrite(RST_PIN, LOW);
    delay(10);
    digitalWrite(RST_PIN, HIGH);
    delay(10);

    while (!rf95.init())
    {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    Serial.println("LoRa radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(FREQ))
    {
        Serial.println("setFrequency failed");
        while (1);
    }

    Serial.print("Set Freq to: ");
    Serial.println(FREQ);
    rf95.setTxPower(23, false);
   delay(100);
    Serial.flush();
    delay(100);

 


}

void loop() {

  // put your main code here, to run repeatedly:
  /*Check for Serial data, ie user inputs*/
  if (Serial.available() > 0)
  {
    delay(50);
    size_t available = Serial.available();
    //delay(100);
    char toSend[available];
    Serial.readBytes(toSend, available);
    Serial.println(toSend);
    Serial.println(Serial.available());
    rf95.send(toSend, available);
    rf95.waitPacketSent();
    Serial.flush();
  }


}
