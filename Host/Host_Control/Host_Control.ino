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


  /**
   * We gonna check for user inputs, ie serial data
   * This data will be sent to the other systems.
   * 
   * The serial data in this case corresponds to specific commands to be sent
   */
  if (Serial.available() > 0) //check to see if the user man inputted anything
  {
    delay(50); //give her a moment to cook incase serial feels slow
    size_t available = Serial.available(); //how big is this gonna be
    //delay(100);
    
    uint8_t toSend[available];
     uint8_t data[] = "And hello back to you";
    //Serial.println(Serial.available());
    //Serial.println(RH_RF95_MAX_MESSAGE_LEN, DEC);
    Serial.readBytes(toSend, available);
    //Serial.println(toSend);
    //Serial.println(Serial.available());
    //call a build packet/command function here!
    rf95.send(toSend, available); 
    rf95.waitPacketSent(2000);
    
    Serial.flush();
  }


  /**
   * now we are going to check and see if our friends the
   * motor controller and ground support system want to tell us how pretty
   * we are
   */

   //is a message there?
   if (rf95.available())
  {
    // Should be a message for us now   
    char buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    }
  }
   


}
