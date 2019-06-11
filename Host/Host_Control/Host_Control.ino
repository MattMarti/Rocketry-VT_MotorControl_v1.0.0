#include "Host_Config.h"
#include <CircularBuffer.h>
#include <SPI.h>
#include <RH_RF95.h>
//#include <packet_interpret.h>

#include "HC_Commands.h"


//GS = Ground Support
//MC = Motor Controller
//HC = Host Computer (this)


using namespace HC_Commands;
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

/**
    Listens for a short time and stores all data from the radio,
    up to the maximum size of the buffer.
*/
void radio_recieve(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer);

void readPacket(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer);

void actOn(uint8_t packdata[], int packSize);

boolean sameAs(uint8_t data[], uint8_t target[]);

/*
 * state variables
 */
// bool readyToTrans = false; //need all unlocks = true
 bool readyToFill = false;  //need readyToTrans = true and MC and GS to be ready
 bool readyToLaunch = false; //need feedline disconnected, tank filled, and above true
 //bool unlock1 = false; 
// bool unlock2 = false;
// bool unlock3 = false;
 bool MCready = false; //is motor ready
 bool GSready = false;
 bool tankFull = false;
 bool feedDisconn = false;
 

/*Functions to checkand set values for predicates*/
//void checkReadyToTrans();
void checkReadyToFill();
void checkReadyToLaunch();


 /*functions to ping GS and MC states, and all */
 void pingMCstate();
 void pingGSstate();
 void pingHCstate();
 void pingAll();



void setup() {
  // put your setup code here, to run once:
  UserInput.begin(BAUD); //begin comms for user inputting (Serial)

  Serial.println("2019 Rocketry at VT Launch Control System: Host");



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

  Serial.println("To Show Commands Type in HEX: 0x00 0x03 0xFF");
  delay(100);
  Serial.flush();
  delay(100);




}

void loop() {


  // put your main code here, to run repeatedly:


  /**
     We gonna check for user inputs, ie serial data
     This data will be sent to the other systems.
     The serial data in this case corresponds to specific commands to be sent
  */
  if (Serial.available() > 0) //check to see if the user man inputted anything
  {
   
    delay(50); //give her a moment to cook incase serial feels slow

    serial_receive(serialBuf);

    CircularBuffer<uint8_t, BUFFER_SIZE> serialPacket = parse_serial_packet(serialBuf);
 
    
    if (serialPacket.size() > 0)
    {
//delay(100);

    serialPacket.shift();
    //delay(100);
   
    uint8_t data = serialPacket.shift();
   
    serialPacket.shift();
   
    packetBuilder(data);
  
    rf95.waitPacketSent(200);

    }
    

    
    //rf95.send(forSend, 3);
    

    Serial.flush();

  }


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
     // Serial.println("Got Reply: ");
      for (int i = 0; i < len && buffer.available() > 0; i++)
      {
       // Serial.println(buf[i], HEX);
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
    Serial.println(toSend[i], HEX);
  }


  if (bufferSize > 0)
  {
   // uint8_t dataLen = toSend[2];
   // uint8_t ID = toSend[3];
   // uint8_t checkSumBytes[2];
   // uint8_t packData[dataLen];
   // for (int i = 0; i < dataLen; i++ )
   // {
   //   packData[i] = toSend[(4 + i)];

   // }
    //checkSumBytes[0] = toSend[(4 + dataLen)];
    //checkSumBytes[1] = toSend[(5 + dataLen)];
    //Serial.println("Fire Torpedoes!");

    //Serial.println(toSend[0], HEX);
    //Serial.println(packData[0], HEX);

    //interpret packet with another function

    //Serial.println("size of");
    //Serial.println(sizeof(toSend), DEC);
    //actOn(toSend);
    //Serial.println("buf size");
    //Serial.println(bufferSize, DEC);

    actOn(toSend, bufferSize);


  }
}

void serial_receive(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer)
{
    while (buffer.available() > 0 && Serial.available() > 0)
    {
      
        buffer.push(Serial.read());
    }
}

CircularBuffer<uint8_t, BUFFER_SIZE> parse_serial_packet(CircularBuffer<uint8_t, BUFFER_SIZE> &buf)
{
    bool parsing = true;
    CircularBuffer<uint8_t, BUFFER_SIZE> packet;
    if (buf.size() < 3)
    {
      parsing = false;
    }
    while (parsing && buf.size() > 0)
    {
   
        while (buf.size() > 0 && buf.first() != 0x00)
        {
            buf.shift();
         
        }

        uint8_t length = 3;

        if (buf.size() < length)
        {
            parsing = false;
           //break;
            continue;
        }

        if (buf[2] != 0xFF)
        {
            buf.shift();
            continue;
        }

//        uint8_t length = 3;
//        if (buf.size() < length)
//        {
//            parsing = false;
//            continue;
//        }

        for (int i = 0; i < length; ++i)
        {
            packet.push(buf.shift());
        }
        return packet;
    }
   
    
    return packet;
}

void packetBuilder(uint8_t packet){


  switch (packet) {

    case SHOW_COMMANDS: 

    printCommands();
                  break;
//    case UNLOCK1:  
//   
//                  //unlock1 = true;
//                  Serial.println("unlocked lock 1");
//                   break;
//    case UNLOCK2: 
//   
//                   //unlock2 = true;
//                   Serial.println("unlocked lock 2");
//                   break;
//    case UNLOCK3: 
//;
//                 // unlock3 = true;
//                   Serial.println("unlocked lock 3");
//                  break;
    case PING_STATE_All: 
  
                    pingAll();
                    Serial.println("Pinged State for ALL");
                       
                     break;
    case PING_STATE_MC: 

                     pingMCstate();
                     Serial.println("Pinged MC State");
                     break;
                       
    case PING_STATE_GS: 
   
  
                     pingGSstate();
                     Serial.println("Pinged GS State");
                     break;

    case PING_STATE_HC:
    
                    pingHCstate();
                    Serial.println("Pinged HC State");
                    break;
               
            
    case FILL: 
                      checkReadyToFill();
                      if (readyToFill)
                      {
                     rf95.send(FILL_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("Sent Fill Command");
                      }
                      else
                      {
                        Serial.println("Fill Locked");
                      }
                     break;
    case DISCONNECT_FILL: 
                  //  checkReadyToTrans();
                   // if (readyToTrans)
                   // {
                     rf95.send(DISCONNECT_FILL_PACKET, 7);
                     rf95.waitPacketSent(200);
//                    }
//                    else
//                    {
//                      Serial.println("transmitting locked");
//                    }
                     break;
    case LAUNCH:
                      checkReadyToLaunch();
                      if (readyToLaunch)
                      { 
  
                    rf95.send(LAUNCH_PACKET, 7);
                    rf95.waitPacketSent(200);
    Serial.println("launch sent");
                      }
                      else
                      {
                        Serial.println("Launch Locked!");
                      }
   
 
                     break;
    case MAKE_READY:
//                   checkReadyToTrans();
//                   if (readyToTrans)
//                   {
                    rf95.send(MAKE_READY_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("Sent Make Ready Command");
//                   }
//                   else
//                   {
//                    Serial.println("DENIED: Transmitting Locked");
//                   }
                   break;
    case PAUSE:
//                checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(PAUSE_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("Sent PAUSE Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
    case ABORT:
              rf95.send(ABORT_PACKET, 7);
              rf95.waitPacketSent(200);
              Serial.println("Sent abort Command"); 
              break;              
    case VENT_MANUAL:
//               checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(VENT_MANUAL_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("Sent MANUAL VENT Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
     case PING_TANK_PRESSURE:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(PING_TANK_PRESSURE_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("PINGED TANK PRESSURE Command");
//                }
//                else
//                {
                  Serial.println("Transmissions locked");
               // }
                break;
      case RECC_LAUNCH_PROCEDURE:
                explain();
                break;
      case PING_CHAMBER_PRESSURE:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(PING_CHAMBER_PRESSURE_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("PINGED CHAMBER PRESSURE Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
       case PING_INJ_TEMP:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(PING_INJ_TEMP_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("PINGED INJ TEMP Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
        case PING_NOZ_TEMP:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(PING_NOZ_TEMP_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("PINGED NOZ TEMP Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
        case PING_ALL_DATA:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(PING_ALL_DATA_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("PINGED ALL DATA Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
         case ENABLE_AUTO_VENT:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(ENABLE_AUTO_VENT_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("ENABLE AUTO VENT Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
         case DISABLE_AUTO_VENT:
//                  checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(DISABLE_AUTO_VENT_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("DISABLE AUTO Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
        case PING_MC_SETTINGS:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(PING_MC_SETTINGS_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("PING MC SETTINGS Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
          case CLOSE_VENT:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(CLOSE_VENT_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("SENT CLOSE VENT Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
        case RESUME:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(RESUME_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("Sent RESUME Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
         case ENABLE_WARNINGS:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(ENABLE_WARNINGS_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("WARNINGS ENABLED Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
          case DISABLE_WARNINGS:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(DISABLE_WARNINGS_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("WARNINGS DISABLED Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
          case DECLARE_TANK_FULL:
                 //checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(DECLARE_TANK_FULL_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("Sent TANK DECLARED FULL Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
          case DECLARE_TANK_NOT_FULL:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(DECLARE_TANK_NOT_FULL_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("Sent TANK NOT FULL Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
          case DISABLE_FLOAT_SWITCH:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(DISABLE_FLOAT_SWITCH_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("DISABLED FLOAT SWITCH Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
           case ENABLE_FLOAT_SWITCH:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(ENABLE_FLOAT_SWITCH_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("ENABLED FLOAT SWITCH Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
           case CHECK_GS_CONTINUITY:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(CHECK_GS_CONTINUITY_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("CHECK GS CONTINUITY Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
            case CHECK_MC_CONTINUITY:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(CHECK_MC_CONTINUITY_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("CHECK MC CONTINUITY Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
         case CHECK_MC_FLOAT:
//                 checkReadyToTrans();
//                if (readyToTrans)
//                {
                  rf95.send(CHECK_MC_FLOAT_PACKET, 7);
                     rf95.waitPacketSent(200);
                     Serial.println("CHECK FLOAT Command");
//                }
//                else
//                {
//                  Serial.println("Transmissions locked");
//                }
                break;
                


    
  }
 
}

void actOn(uint8_t packdata[], int psize)
{

     
  if (sameAs(packdata, MC_FILL_STATE, psize, 7))
  {
    Serial.println("MC FILL STATE");
  }
  else if (sameAs(packdata, MC_IDLE_STATE, psize, 7))
  {
    Serial.println("MC IDLE STATE");
    MCready = false;
  }
  else if (sameAs(packdata, MC_READY_STATE, psize, 7))
  {
   
    Serial.println("MC READY STATE");
    MCready = true;
  }
 else if (sameAs(packdata, MC_LAUNCH_STATE, psize, 7))
  {
     
    Serial.println("MC LAUNCH STATE");
    
    
  }
 else if (sameAs(packdata, GS_FILL_STATE, psize, 7))
  {
    Serial.println("GS Fill State");
  }
 else if (sameAs(packdata, GS_IDLE_STATE, psize, 7))
  {
    Serial.println("GS IDLE State");
    GSready = false;
  }
 else if (sameAs(packdata, GS_READY_STATE, psize, 7))
  {
    Serial.println("GS READY State");
    GSready = true;
  }
 else if (sameAs(packdata, GS_LAUNCH_STATE, psize, 7))
  {
    Serial.println("GS LAUNCH State");
  }
 else if (sameAs(packdata, MC_TANK_FULL_STATE, psize, 7))
  {
    Serial.println("MC TANK IS FULL State");
    tankFull = true;
  }

  else if (sameAs(packdata, MC_ABORT_STATE, psize, 7))
  {
    Serial.println("MC ABORT State");
    MCready = false;
    tankFull = false;
  }

  else if (sameAs(packdata, TANK_PRES_PACKET, psize, 8))
  {
    float pres = 14.0538*packdata[5];
    Serial.print("OX TANK PRESSURE IS (psi): ");
    Serial.println(pres, DEC);
  }

else if (sameAs(packdata, CHAMB_PRES_PACKET, psize, 8))
  {
    float pres = 14.0538*packdata[5];
    Serial.print("CHAMBER PRESSURE IS (psi): ");
    Serial.println(pres, DEC);
  }

  else if (sameAs(packdata, INJ_TEMP_PACKET, psize, 8))
  {
    float temp = 10*packdata[5];
    Serial.print("Injector Casing Temp (F): ");
    Serial.println(temp, DEC);
  }

 else if (sameAs(packdata, NOZ_TEMP_PACKET, psize, 8))
  {
    float temp = 10*packdata[5];
    Serial.print("Nozzle Casing Temp (F): ");
    Serial.println(temp, DEC);
  }

 else if (sameAs(packdata, ALL_DATA_PACKET, psize, 11))
  {
       float Oxpres = 14.0538*packdata[5];
    Serial.print("OX TANK PRESSURE IS (psi): ");
    Serial.println(Oxpres, DEC);
     float chambPres = 14.0538*packdata[6];
    Serial.print("CHAMBER PRESSURE IS (psi): ");
    Serial.println(chambPres, DEC);
    float Injtemp = 10*packdata[7];
    Serial.print("Injector Casing Temp (F): ");
    Serial.println(Injtemp, DEC);
    float Noztemp = 10*packdata[8];
    Serial.print("Nozzle Casing Temp (F): ");
    Serial.println(Noztemp, DEC);
   
  }

 else  if (sameAs(packdata, AUTO_VENT_ENABLED, psize, 7))
  {
    Serial.println("Auto Vent Enabled");
  }

 else  if (sameAs(packdata, AUTO_VENT_DISABLED, psize, 7))
  {
    Serial.println("Auto Vent Disabled");
  }

 else  if (sameAs(packdata, MC_AUTO_PAUSED, psize, 7))
  {
    Serial.println("MC AUTOPAUSED");
    MCready = false;
  }

 else  if (sameAs(packdata, WARNINGS_ENABLED, psize, 7))
  {
    Serial.println("WARNINGS ENABLED");
  }

 else  if (sameAs(packdata, WARNINGS_DISABLED, psize, 7))
  {
    Serial.println("WARNINGS DISABLED");
  }

 else  if (sameAs(packdata, MC_MAN_PAUSED, psize, 7))
  {
    Serial.println("MC MANUALLY PAUSED");
  }

 else  if (sameAs(packdata, MC_OX_OVER_PRES_WARNING, psize, 7))
  {
    Serial.println("WARNING: Pressure too high in oxidizer tank");
     rf95.send(PING_TANK_PRESSURE_PACKET, 7);
                     rf95.waitPacketSent(200);
  }

 else  if (sameAs(packdata, MC_INJ_OVER_TEMP_WARNING, psize, 7))
  {
    Serial.println("WARNING: Temp too high over injector");
     rf95.send(PING_INJ_TEMP_PACKET, 7);
                     rf95.waitPacketSent(200);
  }

 else  if (sameAs(packdata, MC_CHAMB_OVER_PRES_WARNING, psize, 7))
  {
    Serial.println("WARNING: Pressure too high in CHAMBER");
     rf95.send(PING_CHAMBER_PRESSURE_PACKET, 7);
                     rf95.waitPacketSent(200);
  }

 else  if (sameAs(packdata, MC_NOZ_OVER_TEMP_WARNING, psize, 7))
  {
    Serial.println("WARNING: Temp too high over nozzle");
     rf95.send(PING_NOZ_TEMP_PACKET, 7);
                     rf95.waitPacketSent(200);
  }

  else if (sameAs(packdata, MC_LOW_PRES_DROP_WARNING, psize, 7))
  {
    Serial.println("WARNING: LOW PRESSURE DROP ACROSS INJECTOR");
    rf95.send(PING_TANK_PRESSURE_PACKET, 7);
                     rf95.waitPacketSent(200);
                     delay(100);
                       rf95.send(PING_CHAMBER_PRESSURE_PACKET, 7);
                     rf95.waitPacketSent(200);
  }

  else  if (sameAs(packdata, MC_CONTINUITY_GOOD, psize, 7))
  {
    Serial.println("MC CONTINUITY GOOD");
     
  }

  else  if (sameAs(packdata, MC_NO_CONT_VENT, psize, 7))
  {
    Serial.println("NO CONTINUITY ON VENT");
     
  }

 else   if (sameAs(packdata, MC_NO_CONT_OX_VALVE, psize, 7))
  {
    Serial.println("NO CONTINUITY ON OX VALVE");
     
  }

 else   if (sameAs(packdata, MC_NO_CONT_EMATCH, psize, 7))
  {
    Serial.println("NO CONTINUITY EMATCH");
     
  }

  else  if (sameAs(packdata, GS_ABORT_STATE, psize, 7))
  {
    Serial.println("GS ABORT STATE");
     GSready = false;
  }

 else   if (sameAs(packdata, GS_PAUSED, psize, 7))
  {
    Serial.println("GS PAUSED");
     
  }

  else  if (sameAs(packdata, GS_DISCONNECTED, psize, 7))
  {
    Serial.println("FILL LINE DISCONNECTED");
    feedDisconn = true;
    
  }

  else  if (sameAs(packdata, GS_CONTINUITY_GOOD, psize, 7))
  {
    Serial.println("GS CONTINUITY GOOD");
     
  }

  else  if (sameAs(packdata, GS_NO_CONT_SOLENOID, psize, 7))
  {
    Serial.println("NO CONTINUITY ON SOLENOID");
     
  }

  else  if (sameAs(packdata, GS_TANK_FULL_STATE, psize, 7))
  {
    Serial.println("GS TANK FULL STATE");
     
  }

 else   if (sameAs(packdata, GS_NO_CONT_ACT, psize, 7))
  {
    Serial.println("NO CONTINUITY ON LINEAR ACTUATOR");
     
  }

 else   if (sameAs(packdata, GS_DISCONNECTING, psize, 7))
  {
    Serial.println("FEEDLINE DISCONNECTING: PLEASE ALLOW 1 MINUTE TO PASS");
 
  }

 else   if (sameAs(packdata, VENTING_ON, psize, 7))
  {
    Serial.println("VENTING");
     
  }

 else  if (sameAs(packdata, VENTING_OFF, psize, 7))
  {
    Serial.println("VENT CLOSED");
     
  }

  
  

  
    
}


boolean sameAs(uint8_t data[], uint8_t target[], int L1, int L2)
{
//Serial.println("in same As");
//Serial.println(sizeof(data), DEC);
 // Serial.println(sizeof(data), DEC);
 if (L1 == 7)
 {
  //Serial.println("size is 7");
  if (L1 != L2)
  {
    //Serial.println("not same size");
    return false;
    //Serial.println("Fuck you");
  }
  else
  {
   // Serial.println("same size");
    for (int i = 0; i < L1; i++)
    {
      if (data[i]!=target[i])
      {
       // Serial.println("not equal");
        
        return false;
      }
    }
    //Serial.println("equal");
    return true;
  }
 }
 else if (L1 > 7)
 {
  //Serial.println("more than 7");
  for (int i = 0; i < 4; i++)
    {
      if (data[i]!=target[i])
      {
        return false;
      }
    }
    for (int i = L1; i < 2; i--)
    {
      if (data[i]!=target[i])
      {
        return false;
      }
    }
    return true;
 }
 else
 {
  return false;
 }
}




// bool readyToTrans = false; //need all unlocks = true
// bool readyToFill = false;  //need readyToTrans = true and MC and GS to be ready
// bool readyToLaunch = false; //need feedline disconnected, tank filled, and above true
// bool unlock1 = false; 
// bool unlock2 = false;
// bool unlock3 = false;

/*Functions to checkand set values for predicates*/
//void checkReadyToTrans()
//{
//  if (!readyToTrans)
//  {
//     if (unlock1 && unlock2 &&unlock3)
//     {
//      readyToTrans = true;
//      //Serial.println("Ready to transmit commands");
//     }
//     
//  }
// 
//}
void checkReadyToFill()
{
  if (!readyToFill)
  {
      pingMCstate();
      //read whats on the radio
      //radio_recieve(LoRaBuf);
      CircularBuffer<uint8_t, BUFFER_SIZE> radPac = parse_packet(LoRaBuf);
      readPacket(radPac);
       pingGSstate();
      //read whats on the radio
      //radio_recieve(LoRaBuf);
       radPac = parse_packet(LoRaBuf);
      readPacket(radPac);
      if (GSready && MCready)
      {
       // Serial.println("ready to fill");
       readyToFill = true;
      }
     
      
  }
 
}
void checkReadyToLaunch()
{
  if (!readyToLaunch)
  {
      pingMCstate();
      //read whats on the radio
      radio_recieve(LoRaBuf);
      CircularBuffer<uint8_t, BUFFER_SIZE> radPac = parse_packet(LoRaBuf);
      readPacket(radPac);
       pingGSstate();
      //read whats on the radio
      radio_recieve(LoRaBuf);
      radPac = parse_packet(LoRaBuf);
      readPacket(radPac);
      if (readyToFill && feedDisconn && tankFull)
      {
      //  Serial.println("ready to launch");
      readyToLaunch = true;
      }
    
      
  }
  
}


 /*functions to ping GS and MC states, and all */
 void pingMCstate()
 {
     rf95.send(PING_STATE_PACKET_MC, 7);
     rf95.waitPacketSent(200);
     radio_recieve(LoRaBuf);
 }
 void pingGSstate()
 {
  rf95.send(PING_STATE_PACKET_GS, 7);
  rf95.waitPacketSent(200);
  radio_recieve(LoRaBuf);
 }
 void pingHCstate()
 {
 
  if (readyToLaunch)
  {
    Serial.println("Launch Unlocked");
  }
  else if (readyToFill)
  {
    Serial.println("Filling Unlocked");
  }
  else
  {
    Serial.println("Launch and Fill Locked");
  }
//  else if (readyToTrans)
//  {
//    Serial.println("Transmitting Commands Unlocked");
//  }
//  else if (unlock1 && unlock2 && unlock3)
//  {
//    readyToTrans = true;
//    Serial.println("Ready to transmit commands");
//  }
//  else
//  {
//  if (unlock3)
//  {
//    Serial.println("Unlocked lock 3");
//  }
//  if (unlock2)
//  {
//    Serial.println("Unlocked lock 2");
//  }
//  if (unlock1)
//  {
//    Serial.println("Unlocked lock 1");
//  }
//  if (!(unlock1 || unlock2 || unlock3))
//  {
//    Serial.println("All Host Transmisson locks locked");
//  }
  //}
  radio_recieve(LoRaBuf);
 }
 
 void pingAll()
 {
  Serial.println("Pinged ALL:");
  pingHCstate();
 
  pingMCstate();
  
  pingGSstate();

 }

