#include "ground_support_config.h"
#include <CircularBuffer.h>
#include <SPI.h>
#include <RH_RF95.h>
//#include <packet_interpret.h>

#include "GS_Commands.h"
#include "GS_hardware.h"





using namespace GS_Commands;
using namespace LoRa;
using namespace User;


RH_RF95 rf95(CS_PIN, INT_PIN);
const size_t BUFFER_SIZE = RH_RF95_MAX_MESSAGE_LEN * 2;

CircularBuffer<uint8_t, BUFFER_SIZE> LoRaBuf, serialBuf;

/**
    Parses the provided buffer for packets, and removes all bytes which were processed.
*/
CircularBuffer<uint8_t, BUFFER_SIZE> parse_packet(CircularBuffer<uint8_t, BUFFER_SIZE> &buf);


CircularBuffer<uint8_t, BUFFER_SIZE> parse_serial_packet(CircularBuffer<uint8_t, BUFFER_SIZE> &buf);

void packetBuilder(uint8_t packet);

uint8_t currentState[7]; //current GS state, starts in idle
uint8_t lastState[7]; //last state before pause

/**
    Listens for a short time and stores all data from the radio,
    up to the maximum size of the buffer.
*/
void radio_recieve(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer);

void readPacket(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer);

void actOn(uint8_t packdata[], int packSize);

void fillFunc();

boolean sameAs(uint8_t data[], uint8_t target[]);

bool fill = false;
bool disconn = false;

unsigned long previousTime = 0;
unsigned long previousTime2 = 0;
void disconnFunc();
bool solOn = true;
uint8_t cont[2] = {0, 0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD); //begin comms for user inputting (Serial)

  Serial.println("2019 Rocketry at VT Launch Control System: Ground Support");

pinsOff();
memcpy(currentState, GS_IDLE_STATE, 7);
memcpy(lastState, GS_IDLE_STATE, 7);


  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  delay(100);
  digitalWrite(RST_PIN, LOW);
  delay(10);
  digitalWrite(RST_PIN, HIGH);
  delay(10);

  if (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    //while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(FREQ))
  {
    Serial.println("setFrequency failed");
    //while (1);
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
     now we are going to check and see if our friends the
     motor controller and ground support system want to tell us how pretty
     we are
  */

  //read whats on the radio
  radio_recieve(LoRaBuf);

  //parse otu a packet
  CircularBuffer<uint8_t, BUFFER_SIZE> fromRadio = parse_packet(LoRaBuf);
 

  readPacket(fromRadio);


  if (fill)
  {
    fillFunc();
  }

  if (disconn)
  {
    disconnFunc();
  }







}



/*
   helper functions
*/
void radio_recieve(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer)
{
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(100))
  {
    if (rf95.recv(buf, &len))
    {
      //Serial.println("Got Reply: ");
      for (int i = 0; i < len && buffer.available() > 0; i++)
      {
      //  Serial.println(buf[i], HEX);
        buffer.push(buf[i]);
      }
    }
  }
}

CircularBuffer<uint8_t, BUFFER_SIZE> parse_packet(CircularBuffer<uint8_t, BUFFER_SIZE> &buf)
{
  bool parsing = true;
  CircularBuffer<uint8_t, BUFFER_SIZE> packet;
//  if (buf.size() < 6)
//  {
//    parsing = false;
//  }
  while (parsing && buf.size() > 0)
  {
    //Serial.println("in firstwhile");
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
    uint8_t packData[dataLen];
    for (int i = 0; i < dataLen; i++ )
    {
      packData[i] = toSend[(4 + i)];

    }
    checkSumBytes[0] = toSend[(4 + dataLen)];
    checkSumBytes[1] = toSend[(5 + dataLen)];
   // Serial.println("Fire Torpedoes!");

   // Serial.println(toSend[0], HEX);
   // Serial.println(packData[0], HEX);

    //interpret packet with another function

    //Serial.println("size of");
    //Serial.println(sizeof(toSend), DEC);
    //actOn(toSend);

    actOn(toSend, bufferSize);


  }
}






void actOn(uint8_t packdata[], int psize)
{
  if (sameAs(packdata, PING_STATE_PACKET_GS, psize, 7))
  {
    Serial.println("sent state");
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
  }
  if (sameAs(packdata, FILL_PACKET, psize, 7))
  {
    Serial.println("GS FILL");
    memcpy(currentState, GS_FILL_STATE, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    fill = true;
    
    
  }
  if (sameAs(packdata, LAUNCH_PACKET, psize, 7))
  {
    solenoidOff();
    actIn();
    memcpy(currentState, GS_LAUNCH_STATE, 7);
    Serial.println("GS LAUNCH");
   
  }


     if (sameAs(packdata, MAKE_READY_PACKET, psize, 7))
  {
    pinsOff(); //everything off
    checkCont(cont);
    if ((cont[0] == 1) && (cont[1] == 1))
    {
       memcpy(currentState, GS_READY_STATE, 7);
       Serial.println("READY");
    }
    else
    {
      if (cont[0] == 0)
      {
        memcpy(currentState, GS_NO_CONT_SOLENOID, 7);
            rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
        
      }
      if (cont[1] == 0)
      {
          memcpy(currentState, GS_NO_CONT_ACT, 7);
            rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
      }
    }
    
   
  }
     if (sameAs(packdata, PAUSE_PACKET, psize, 7))
  {
    pinsOff();
    memcpy(lastState, currentState, 7);
    memcpy(currentState, GS_PAUSED, 7);
  
    Serial.println("PAUSE");
   
  }

     if (sameAs(packdata, ABORT_PACKET, 7, psize))
  { 
    solenoidOff();
    actIn();
    memcpy(currentState, GS_ABORT_STATE, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    Serial.println("Aborted");
   
  }

     if (sameAs(packdata, RESUME_PACKET, 7, psize))
  {
    memcpy(currentState, lastState, 7);
     rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    Serial.println("RESUME AVAILABLE");
   
  }

     if (sameAs(packdata, DECLARE_TANK_FULL_PACKET, 7, psize))
  {
    solenoidOff();
    memcpy(currentState, GS_TANK_FULL_STATE, 7);
     rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    Serial.println("TANK DECLARED FULL");
   
  }

     if (sameAs(packdata, DECLARE_TANK_NOT_FULL_PACKET, 7, psize))
  {
    
    pinsOff();
     memcpy(currentState, GS_READY_STATE, 7);
     rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    Serial.println("DECLARED TANK NOT FULL");
   
  }

     if (sameAs(packdata, CHECK_GS_CONTINUITY_PACKET, 7, psize))
  {
    Serial.println("CHECKING CONTINUITY");
    checkCont(cont);
    if ((cont[0] == 1) && (cont[1] == 1))
    {
       rf95.send(GS_CONTINUITY_GOOD, 7);
    rf95.waitPacketSent(200);
       Serial.println("Continutity Good");
    }
    else
    {
      if (cont[0] == 0)
      {
        
            rf95.send(GS_NO_CONT_SOLENOID, 7);
    rf95.waitPacketSent(200);
        
      }
      if (cont[1] == 0)
      {
         
            rf95.send(GS_NO_CONT_ACT, 7);
    rf95.waitPacketSent(200);
      }
    }
   
  }

       if (sameAs(packdata, MC_TANK_FULL_STATE, 7, psize))
  {
    solenoidOff();
     memcpy(currentState, GS_TANK_FULL_STATE, 7);
     rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    
    Serial.println("MC Delcared Tank Full");
   
  }

       if (sameAs(packdata, MC_ABORT_STATE, 7, psize))
  {
     solenoidOff();
    actIn();
    memcpy(currentState, GS_ABORT_STATE, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    Serial.println("Aborting based on MC abort state");
   
  }

       if (sameAs(packdata, MC_AUTO_PAUSED, 7, psize))
  {
    solenoidOff();
    actIn();
     memcpy(lastState, currentState, 7);
    memcpy(currentState, GS_DISCONNECTING, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    Serial.println("DISCONNECTING FILL");
    disconn = true;
   
  }



        if (sameAs(packdata, DISCONNECT_FILL_PACKET, 7, psize))
  {
    
    memcpy(currentState, GS_PAUSED, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    Serial.println("PAUSED FROM MC AUTO PAUSE");
   
  }


  

  

  
  

    
}


boolean sameAs(uint8_t data[], uint8_t target[], int L1, int L2)
{

 // Serial.println(sizeof(data), DEC);
  if (L1 != L2)


  {
    return false;
    //Serial.println("Fuck you");
  }
  else
  {
    for (int i = 0; i < L1; i++)
    {
      if (data[i]!=target[i])
      {
        return false;
      }
    }
    return true;
  }
}


void fillFunc()
{
  unsigned long interval = 30000; //30 seconds
  unsigned long currTime = millis();
  if ((currTime - previousTime) > interval)
  {
    previousTime = currTime;
    if (solOn)
    {
      solenoidOff();
      solOn = false;
    }
    else
    {
      solenoidOn();
      solOn = true;
    }
  }
}

void disconnFunc()
{
  unsigned long interval = 45000; //45 seconds
  unsigned long currTime = millis();
  if ((currTime - previousTime2) > interval)
  {
    fill = false;
     memcpy(currentState, GS_DISCONNECTED, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    Serial.println("DISCONNECTED FILL");
    
  }
}

