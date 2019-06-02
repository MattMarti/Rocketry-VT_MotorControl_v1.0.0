#include <Host_Config.h>
#include <CircularBuffer.h>
#include <SPI.h>
#include <RH_RF95.h>
//#include <packet_interpret.h>

  using namespace LoRa;
  using namespace User;

  
RH_RF95 rf95(CS_PIN, INT_PIN);
const size_t BUFFER_SIZE = RH_RF95_MAX_MESSAGE_LEN * 2;

CircularBuffer<uint8_t, BUFFER_SIZE> LoRaBuf;

/**
    Parses the provided buffer for packets, and removes all bytes which were processed.
*/
CircularBuffer<uint8_t, BUFFER_SIZE> parse_packet(CircularBuffer<uint8_t, BUFFER_SIZE>& buf);

/**
    Listens for a short time and stores all data from the radio,
    up to the maximum size of the buffer.
*/
void radio_recieve(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer);

void readPacket(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer) ;


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
    //Serial.flush();
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
   
    
    uint8_t forSend[available];
     //uint8_t data[] = "And hello back to you";
    //Serial.println(Serial.available());
    //Serial.println(RH_RF95_MAX_MESSAGE_LEN, DEC);
    Serial.readBytes(forSend, available);
    //Serial.println(toSend);
    //Serial.println(Serial.available());
    
    //call a build packet/command function here!
    rf95.send(forSend, available); 
    rf95.waitPacketSent(2000);
    
    Serial.flush();
  }


  /**
   * now we are going to check and see if our friends the
   * motor controller and ground support system want to tell us how pretty
   * we are
   */

   //read whats on the radio
   radio_recieve(LoRaBuf);

    //parse otu a packet
    CircularBuffer<uint8_t, BUFFER_SIZE> fromRadio = parse_packet(LoRaBuf);
    
    readPacket(fromRadio);

   

//   //is a message there?
//   if (rf95.available())
//  {
//    // Should be a message for us now   
//    char buf[RH_RF95_MAX_MESSAGE_LEN];
//    //Serial.println((int)buf);
//    //char buf[0];
//    uint8_t len = sizeof(buf);
//    
//    if (rf95.recv(buf, &len))
//    {
//      //insert hub code to decode message
//      RH_RF95::printBuffer("Received: ", buf, len);
//      
//      
//     
//    }
//  }
   


}



/*
 * helper functions
 */
 void radio_recieve(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer)
{
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.waitAvailableTimeout(100))
    {
        if (rf95.recv(buf, &len))
        {
            Serial.println("Got Reply: ");
            for (int i = 0; i < len && buffer.available() > 0; i++)
            {
                Serial.println(buf[i], HEX);
                buffer.push(buf[i]);
            }
        }
    }
}

CircularBuffer<uint8_t, BUFFER_SIZE> parse_packet(CircularBuffer<uint8_t, BUFFER_SIZE> &buf)
{
    bool parsing = true;
    CircularBuffer<uint8_t, BUFFER_SIZE> packet;
    while (parsing && buf.size() > 0)
    {
        Serial.println("in firstwhile");
        while (buf.size() > 0 && buf.first() != 0xAA)
        {
            buf.shift();
        }

        if (buf.size() < 2)
        {
            continue;
        }

        if (buf[1] != 0x14)
        {
            buf.shift();
            continue;
        }

        if (buf.size() < 4)
        {
            parsing = false;
            continue;
        }

        uint8_t length = buf[2];
        if (buf.size() < 6 + length)
        {
            parsing = false;
            continue;
        }

        for (int i = 0; i < 6 + length; ++i)
        {
            packet.push(buf.shift());
        }
        return packet;
    }
    return packet;
}

/*we gonna use this to send the packet to be analyzied and commanded*/
void readPacket(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer) 
{
    uint8_t toSend[buffer.size()];
    const size_t bufferSize = buffer.size();
    for (size_t i = 0; buffer.size() > 0; i++) //lets use this loop to store info, notice that this will clear the buffer
    {
        toSend[i] = buffer.shift();
    }
    

    if (bufferSize > 0)
    {
        uint8_t dataLen = toSend[2];
        uint8_t ID = toSend[3];
        uint8_t checkSumBytes[2];
        uint8_t packData[18];
         for (int i = 0; i < dataLen; i++ )
            {
              packData[i] = toSend[(4 + i)];
              

             }
               checkSumBytes[0] = toSend[(4 + dataLen)];
              checkSumBytes[1] = toSend[(5 + dataLen)];
              Serial.println("Fire Torpedoes!");

              Serial.println(toSend[0], HEX);
              Serial.println(packData[0], HEX);

          //send 

    
    }
}


