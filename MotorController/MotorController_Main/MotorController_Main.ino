#include "motor_controller_config.h"
#include <CircularBuffer.h>
#include <SPI.h>
#include <RH_RF95.h>
//#include <packet_interpret.h>

#include "MC_Commands.h"
#include "MC_Hardware.h"




using namespace MC_Commands;
using namespace LoRa;
using namespace User;
using namespace MC_Pins;



RH_RF95 rf95(CS_PIN, INT_PIN);
const size_t BUFFER_SIZE = RH_RF95_MAX_MESSAGE_LEN * 2;

CircularBuffer<uint8_t, BUFFER_SIZE> LoRaBuf, serialBuf;

/**
    Parses the provided buffer for packets, and removes all bytes which were processed.
*/
CircularBuffer<uint8_t, BUFFER_SIZE> parse_packet(CircularBuffer<uint8_t, BUFFER_SIZE> &buf);


CircularBuffer<uint8_t, BUFFER_SIZE> parse_serial_packet(CircularBuffer<uint8_t, BUFFER_SIZE> &buf);

void packetBuilder(uint8_t packet);
void warnFunc(uint8_t oxPres, uint8_t chambPres, uint8_t injTemp, uint8_t nozTemp);
void pauseFunc(uint8_t oxPres, uint8_t chambPres, uint8_t injTemp, uint8_t nozTemp);
void ventFunc(uint8_t oxPres);
uint8_t currentState[7];

/**
    Listens for a short time and stores all data from the radio,
    up to the maximum size of the buffer.
*/
void radio_recieve(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer);

void readPacket(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer);

void actOn(uint8_t packdata[], int packSize);

boolean sameAs(uint8_t data[], uint8_t target[]);

uint8_t lastState[7];

bool filling = false;
bool venting = false;
bool logData = false;
//bool watch = false;
bool warn = true;
bool autoPause = false;
bool autoVent = false;
bool tankFillOvrRide = false;
bool tankEmptyOvrRide = false;
bool floatEnabled = true;


void setup() {
  
  // put your setup code here, to run once:
  UserInput.begin(BAUD); //begin comms for user inputting (Serial)
 pinMode(FLOAT_SWITCH_PIN, INPUT_PULLUP);
  //Serial.println("2019 Rocketry at VT Launch Control System: Motor Controller");

memcpy(currentState, MC_IDLE_STATE, 7);
memcpy(lastState, currentState, 7);
pinsOff();


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
    while (1);
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

  if (filling)
  {
    if (tankFull() && !tankEmptyOvrRide && floatEnabled)
    {
     memcpy(currentState, MC_TANK_FULL_STATE, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    closeOxValve();
    closeVent();
    filling = false;
    Serial.println("MC tank full");
    }
    else if (tankFillOvrRide)
    {
      memcpy(currentState, MC_TANK_FULL_STATE, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    closeOxValve();
    closeVent();
    filling = false;
    Serial.println("MC tank full");
    }
  }

  if (logData)
  {
    uint8_t oxPres = readOxPressure();
    uint8_t chamberPres = readChamberPressure();
    uint8_t injTemp = readInjTemp()*10;
    uint8_t nozTemp = readNozTemp()*10;
    Serial.print("Ox Tank Pressure (bar): ");
    Serial.println(oxPres, DEC);
    Serial.print("Chamber Pressure (bar): ");
    Serial.println(chamberPres, DEC);
    Serial.print("Injector Casing Temp (F): ");
    Serial.println(injTemp, DEC);
    Serial.print("Nozzle Casing Temp (F): ");
    Serial.println(nozTemp, DEC);
    if (warn)
    {
      warnFunc(oxPres, chamberPres, injTemp, nozTemp);
    }
    if (autoPause)
    {
      pauseFunc(oxPres, chamberPres, injTemp, nozTemp);
    }
    if (autoVent)
    {
      ventFunc(oxPres);
    }
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
   // uint8_t dataLen = toSend[2];
  //  uint8_t ID = toSend[3];
   // uint8_t checkSumBytes[2];
   // uint8_t packData[dataLen];
   // for (int i = 0; i < dataLen; i++ )
   // {
     // packData[i] = toSend[(4 + i)];

   // }
  //  checkSumBytes[0] = toSend[(4 + dataLen)];
   // checkSumBytes[1] = toSend[(5 + dataLen)];
    //Serial.println("Fire Torpedoes!");

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
  if (sameAs(packdata, PING_STATE_PACKET_MC, 7, psize))
  {
    Serial.println("sent state");
    
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    delay(100);
    if (venting)
    {
      rf95.send(VENTING_ON, 7);
    rf95.waitPacketSent(200);
    delay(100);
    }
    else
    {
      rf95.send(VENTING_OFF, 7);
    rf95.waitPacketSent(200);
    delay(100);
    }
  }
  if (sameAs(packdata, FILL_PACKET, 7, psize))
  {
    memcpy(currentState, MC_FILL_STATE, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    closeOxValve();
    crackVent();
    venting = true;
    filling = true;
    logData = true;
    Serial.println("MC FILL");
    
  }
  if (sameAs(packdata, LAUNCH_PACKET, 7, psize))
  {
     memcpy(currentState, MC_LAUNCH_STATE, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    closeVent();
    fireEmatch();
    openOxValve();
    venting = false;
    filling = false;
    Serial.println("MC LAUNCHED!");
   
    
 
  }
  if (sameAs(packdata, MAKE_READY_PACKET, 7, psize))
  {
    venting = false;
    filling = false;
    pinsOff();
    int8_t cont[3] = {0, 0, 0};
   checkCont(cont);
   if (cont[0] == 0)
   {
     rf95.send(MC_NO_CONT_VENT, 7);
    rf95.waitPacketSent(200);
   }
   else if (cont[1] == 0)
   {
      rf95.send(MC_NO_CONT_OX_VALVE, 7);
    rf95.waitPacketSent(200);
   }
   else if (cont[2] == 0)
   {
     rf95.send(MC_NO_CONT_EMATCH, 7);
    rf95.waitPacketSent(200);
   }
   else
   {
     memcpy(currentState, MC_READY_STATE, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
   }
    closeOxValve();
    closeVent();
  }

   if (sameAs(packdata, DISCONNECT_FILL_PACKET, 7, psize))
  {
    venting = false;
    filling = false;
    pinsOff();
    closeVent();
    closeOxValve();
    //memcpy(currentState, MC_READY_STATE, 7);
    //rf95.send(currentState, 7);
    //rf95.waitPacketSent(200);
    Serial.println("MC Disconn FILL");
    
  }

   if (sameAs(packdata, PAUSE_PACKET, 7, psize))
  {
    pinsOff();
     memcpy(lastState, currentState, 7);
    memcpy(currentState, MC_MAN_PAUSED, 7);
  
  
    Serial.println("MC PAUSED");
    
  }

   if (sameAs(packdata, ABORT_PACKET, 7, psize))
  {
    pinsOff();
    openVentFull();
   closeOxValve();
   venting = false;
   filling = false;
   logData = false;
    memcpy(currentState, MC_ABORT_STATE, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    Serial.println("MC ABORT");
    
  }

   if (sameAs(packdata, VENT_MANUAL_PACKET, 7, psize))
  {
    crackVent();
    venting = true;
    
    Serial.println("MC MANUAL VENT");
    
  }

   if (sameAs(packdata, PING_TANK_PRESSURE_PACKET, 7, psize))
  {
    uint8_t pres = readOxPressure();
    uint8_t packet[8] = {0xAA, 0x14, 0x02, 0xCC, 0x47, pres, 0xDD, 0xE1}; 
    rf95.send(packet, 8);
    rf95.waitPacketSent(200);
    Serial.println("PING TANK PRES");
    
  }

   if (sameAs(packdata, PING_CHAMBER_PRESSURE_PACKET, 7, psize))
  {
    uint8_t pres = readChamberPressure();
    uint8_t packet[8] = {0xAA, 0x14, 0x02, 0xCC, 0x48, pres, 0xDF, 0xE7};
    rf95.send(packet, 8);
    rf95.waitPacketSent(200);
   
    Serial.println("PING CHAMBER PRESSURE");
    
  }

   if (sameAs(packdata, PING_INJ_TEMP_PACKET, 7, psize))
  {
      uint8_t temp = readInjTemp();
    uint8_t packet[8] = {0xAA, 0x14, 0x02, 0xCC, 0x49, temp, 0xDF, 0xE7};
    rf95.send(packet, 8);
    rf95.waitPacketSent(200);
    Serial.println("INJ TEMP");
    
  }

   if (sameAs(packdata, PING_NOZ_TEMP_PACKET, 7, psize))
  {
    uint8_t temp = readNozTemp();
    uint8_t packet[8] = {0xAA, 0x14, 0x02, 0xCC, 0x50, temp, 0xD3, 0xB7}; 
    rf95.send(packet, 8);
    rf95.waitPacketSent(200);
    Serial.println("NOZ TEMP");
    
  }

   if (sameAs(packdata, PING_ALL_DATA_PACKET, 7, psize))
  {
    uint8_t oxPres = readOxPressure();
    uint8_t chamberPres = readChamberPressure();
    uint8_t injTemp = readInjTemp();
    uint8_t nozTemp = readNozTemp();

    uint8_t packet[11] = {0xAA, 0x14, 0x04, 0xCC, 0x51, oxPres, chamberPres, injTemp, nozTemp, 0xD9, 0x9D}; 
    rf95.send(packet, 11);
    rf95.waitPacketSent(200);
    Serial.println("ALL DATA");
    
  }

   if (sameAs(packdata, ENABLE_AUTO_VENT_PACKET, 7, psize))
  {
    autoVent = true;
    Serial.println("AUTO VENT ENABLED");
    
  }

   if (sameAs(packdata, DISABLE_AUTO_VENT_PACKET, 7, psize))
  {
    autoVent = false;
    Serial.println("MC AUTO VENT DISABLED");
    
  }

   if (sameAs(packdata, ENABLE_AUTO_PAUSE_PACKET, 7, psize))
  {
    autoPause = true;
    Serial.println("MC AUTO PAUSE ENABLED");
    
  }

   if (sameAs(packdata, DISABLE_AUTO_PAUSE_PACKET, 7, psize))
  {
    autoPause = false;
    Serial.println("MC AUTOPAUSE DISABLED");
    
  }

   if (sameAs(packdata, PING_MC_SETTINGS_PACKET, 7, psize))
  {
    if (autoVent)
    {
      rf95.send(AUTO_VENT_ENABLED, 7);
    rf95.waitPacketSent(200);
    delay(100);
    }
    else
    {
      rf95.send(AUTO_VENT_DISABLED, 7);
    rf95.waitPacketSent(200);
    delay(100);
    }
    if (autoPause)
    {
      rf95.send(AUTO_PAUSE_ENABLED, 7);
    rf95.waitPacketSent(200);
    delay(100);
    }
    else
    {
      rf95.send(AUTO_PAUSE_DISABLED, 7);
    rf95.waitPacketSent(200);
    delay(100);
    }
    if (warn)
    {
      rf95.send(WARNINGS_ENABLED, 7);
    rf95.waitPacketSent(200);
    delay(100);
    }
    else
    {
      rf95.send(WARNINGS_DISABLED, 7);
    rf95.waitPacketSent(200);
    delay(100);
    }
    if (floatEnabled)
    {
      rf95.send(MC_FLOAT_ENABLED, 7);
    rf95.waitPacketSent(200);
    delay(100);
    }
    else
    {
      rf95.send(MC_FLOAT_DISABLED, 7);
    rf95.waitPacketSent(200);
    delay(100);
    }
    Serial.println("MC SETTINGS PINGED");
    
  }

   if (sameAs(packdata, CLOSE_VENT_PACKET, 7, psize))
  {
    closeVent();
    venting = false;
    Serial.println("VENt CLOSED");
    
  }

   if (sameAs(packdata, ENABLE_WARNINGS_PACKET, 7, psize))
  {
    warn = true;
    Serial.println("MC WARNINGS ENABLED");
    
  }

   if (sameAs(packdata, DISABLE_WARNINGS_PACKET, 7, psize))
  {
    warn = false;
    Serial.println("MC WARNINGS DISABLED");
    
  }

   if (sameAs(packdata, RESUME_PACKET, 7, psize))
  {
    memcpy(currentState, lastState, 7);
     rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    //Serial.println("RESUME AVAILABLE");
    Serial.println("MC RESUME");
    
  }

   if (sameAs(packdata, DECLARE_TANK_FULL_PACKET, 7, psize))
  {
    tankFillOvrRide = true;
    Serial.println("MC TANK DECLARED FULL");
    
  }

   if (sameAs(packdata, DECLARE_TANK_NOT_FULL_PACKET, 7, psize))
  {
    tankEmptyOvrRide = true;
    Serial.println("MC TANK DECLARED NOT FULL");
    
  }

   if (sameAs(packdata, DISABLE_FLOAT_SWITCH_PACKET, 7, psize))
  {
    floatEnabled = false;
    Serial.println("MC FLOAT SWITCH DISABLED");
    
  }

   if (sameAs(packdata, ENABLE_FLOAT_SWITCH_PACKET, 7, psize))
  {
    floatEnabled = true;
    Serial.println("MC FLOAT ENABLED");
    
  }

   if (sameAs(packdata, CHECK_MC_CONTINUITY_PACKET, 7, psize))
  {
    uint8_t cont[3] = {0, 0, 0};
    checkCont(cont);
    if ((cont[0] == 1) && (cont[1] == 1) && (cont[2] == 1))
    {
      rf95.send(MC_CONTINUITY_GOOD, 7);
    rf95.waitPacketSent(200);
    //delay(100);
    }
    else
    {
    if (cont[0] == 0)
    {
      rf95.send(MC_NO_CONT_VENT, 7);
    rf95.waitPacketSent(200);
    delay(100);
    Serial.println("no vent continuity");
    }
    if (cont[1] == 0)
    {
      rf95.send(MC_NO_CONT_OX_VALVE, 7);
    rf95.waitPacketSent(200);
    delay(100);
    Serial.println("no ox continuity");
    }
    if (cont[2] == 1)
    {
         rf95.send(MC_NO_CONT_EMATCH, 7);
    rf95.waitPacketSent(200);
    delay(100);
    Serial.println("no ematch continuity");
    }
    }
    
    Serial.println("MC CHECK CONT");
    
  }

     if (sameAs(packdata, CHECK_MC_FLOAT_PACKET, 7, psize))
  {
    if (tankFull())
  {
    rf95.send(MC_FLOAT_TRIPPED, 7);
    rf95.waitPacketSent(200);
  }
  else
  {
    rf95.send(MC_FLOAT_NOT_TRIPPED, 7);
    rf95.waitPacketSent(200);
  }
  
    Serial.println("MC CHECK FLOAT");
    
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

void warnFunc(uint8_t oxPres, uint8_t chambPres, uint8_t injTemp, uint8_t nozTemp)
{
  float pDiff = ((oxPres - chambPres)/((oxPres + chambPres)/2))*100;
  if (pDiff < 20.0)
  {
    rf95.send(MC_LOW_PRES_DROP_WARNING, 7);
    rf95.waitPacketSent(200);
    delay(100);
  }
  if (oxPres > 63)
  {
    rf95.send(MC_OX_OVER_PRES_WARNING, 7);
    rf95.waitPacketSent(200);
    delay(100);
  }
  if (chambPres > 42)
  {
    rf95.send(MC_CHAMB_OVER_PRES_WARNING, 7);
    rf95.waitPacketSent(200);
    delay(100);
  }
  if (injTemp > 400)
  {
    rf95.send(MC_INJ_OVER_TEMP_WARNING, 7);
    rf95.waitPacketSent(200);
    delay(100);
  }
  if (nozTemp > 400)
  {
    rf95.send(MC_NOZ_OVER_TEMP_WARNING, 7);
    rf95.waitPacketSent(200);
    delay(100);
  }
}

void pauseFunc(uint8_t oxPres, uint8_t chambPres, uint8_t injTemp, uint8_t nozTemp)
{
  float pDiff = ((oxPres - chambPres)/((oxPres + chambPres)/2))*100;
  bool pause = false;
  if (pDiff < 10.0)
  {
   pause = true;
  }
  if (oxPres > 68)
  {
   pause = true;
  }
  if (chambPres > 58)
  {
    pause = true;
  }
  if (injTemp > 460)
  {
    pause = true;
  }
  if (nozTemp > 460)
  {
   pause = true;
  }
  if (pause)
  {
     pinsOff();
    crackVent();
   closeOxValve();
   venting = true;
   filling = false;
   logData = true;
    memcpy(currentState, MC_AUTO_PAUSED, 7);
    rf95.send(currentState, 7);
    rf95.waitPacketSent(200);
    Serial.println("MC AUTO PAUSE");
  }
}

void ventFunc(uint8_t oxPres)
{
  if (oxPres > 63)
  {
    crackVent();
    venting = true;
      rf95.send(VENTING_ON, 7);
    rf95.waitPacketSent(200);
    Serial.println("MC AUTO VENT");
  }
}

