#include <Host_Config.h>
#include <CircularBuffer.h>
#include <SPI.h>
#include <RH_RF95.h>
//#include <packet_interpret.h>

#include <HC_Commands.h>


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
 bool readyToTrans = false; //need all unlocks = true
 bool readyToFill = false;  //need readyToTrans = true and MC and GS to be ready
 bool readyToLaunch = false; //need feedline disconnected, tank filled, and above true
 bool unlock1 = false; 
 bool unlock2 = false;
 bool unlock3 = false;
 bool MCready = false; //is motor ready
 bool GSready = false;
 bool tankFull = false;
 bool feedDisconn = false;
 

/*Functions to checkand set values for predicates*/
void checkReadyToTrans();
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

  Serial.println("2019 Rocketry at VT Launch Control System");



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

  if (rf95.waitAvailableTimeout(500))
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
  }


  if (bufferSize > 0)
  {
   // uint8_t dataLen = toSend[2];
   // uint8_t ID = toSend[3];
   // uint8_t checkSumBytes[2];
   // uint8_t packData[dataLen];
   // for (int i = 0; i < dataLen; i++ )
   // {
     // packData[i] = toSend[(4 + i)];

   // }
    //checkSumBytes[0] = toSend[(4 + dataLen)];
    //checkSumBytes[1] = toSend[(5 + dataLen)];
   // Serial.println("Fire Torpedoes!");

    //Serial.println(toSend[0], HEX);
    //Serial.println(packData[0], HEX);

    //interpret packet with another function

    //Serial.println("size of");
    //Serial.println(sizeof(toSend), DEC);
    //actOn(toSend);

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
  //uint8_t pee[3] = {0x00, 0x21, 0xFF};
//  for (int i =0; i < 7; i++)
//  {
//  Serial.println(LAUNCH_PACKET[i], HEX);
//  }
  switch (packet) {
    case SHOW_COMMANDS: printCommands();
                  break;
    case UNLOCK1:  
                  unlock1 = true;
                   break;
    case UNLOCK2: 
                   unlock2 = true;
                   break;
    case UNLOCK3: 
                  unlock3 = true;
                  break;
    case PING_STATE_All: 
                    pingAll();
                       
                     break;
    case PING_STATE_MC: 
                     pingMCstate();
                     break;
                       
    case PING_STATE_GS: 
                     
                     pingGSstate();
                     break;

    case PING_STATE_HC:
                    pingHCstate();
                    break;
               
            
    case FILL: 
                     rf95.send(FILL_PACKET, 7);
                     rf95.waitPacketSent(200);
                     break;
    case DISCONNECT_FILL: 
                     rf95.send(DISCONNECT_FILL_PACKET, 7);
                     rf95.waitPacketSent(200);
                     break;
    case LAUNCH: 
                    rf95.send(LAUNCH_PACKET, 7);
                    rf95.waitPacketSent(200);
    Serial.println("launch sent");
   
 
                     break;
  }
}

void actOn(uint8_t packdata[], int psize)
{

  
  if (sameAs(packdata, MC_FILL_STATE, 7, psize))
  {
    Serial.println("MC FILL STATE");
  }
  if (sameAs(packdata, MC_IDLE_STATE, psize, 7))
  {
    Serial.println("MC IDLE STATE");
  }
  if (sameAs(packdata, MC_READY_STATE, psize, 7))
  {
    Serial.println("MC READY STATE");
  }
  if (sameAs(packdata, MC_LAUNCH_STATE, psize, 7))
  {
    Serial.println("MC LAUNCH STATE");
  }
  if (sameAs(packdata, GS_FILL_STATE, 7, psize))
  {
    Serial.println("GS Fill State");
  }
  if (sameAs(packdata, GS_IDLE_STATE, 7, psize))
  {
    Serial.println("GS IDLE State");
  }
  if (sameAs(packdata, GS_READY_STATE, 7, psize))
  {
    Serial.println("GS READY State");
  }
  if (sameAs(packdata, GS_LAUNCH_STATE, psize, 7))
  {
    Serial.println("GS LAUNCH State");
  }
    
}


boolean sameAs(uint8_t data[], uint8_t target[], int L1, int L2)
{

 // Serial.println(sizeof(data), DEC);
  if (sizeof(data) != sizeof(target))
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




// bool readyToTrans = false; //need all unlocks = true
// bool readyToFill = false;  //need readyToTrans = true and MC and GS to be ready
// bool readyToLaunch = false; //need feedline disconnected, tank filled, and above true
// bool unlock1 = false; 
// bool unlock2 = false;
// bool unlock3 = false;

/*Functions to checkand set values for predicates*/
void checkReadyToTrans()
{
  if (!readyToTrans)
  {
     if (unlock1 && unlock2 &&unlock3)
     {
      readyToTrans = true;
      Serial.println("Ready to transmit commands");
     }
     else
     {
      Serial.println("Transmit commands locked");
      pingHCstate();
     }
  }
  else
  {
    Serial.println("Ready to transmit commands");
  }
}
void checkReadyToFill()
{
  if (!readyToFill)
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
      if (readyToTrans && GSready && MCready)
      {
        Serial.println("ready to fill");
      }
      else
      {
        Serial.println("Fill locked");
      }
      
  }
  else
  {
    Serial.println("Ready to fill");
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
      if (readyToFill && feedDisconn && tankFull && readyToTrans)
      {
        Serial.println("ready to launch");
      }
      else
      {
        Serial.println("Launch Locked");
      }
      
  }
  else
  {
    Serial.println("Ready to launch");
  }
}


 /*functions to ping GS and MC states, and all */
 void pingMCstate()
 {
     rf95.send(PING_STATE_PACKET_MC, 7);
     rf95.waitPacketSent(200);
     
 }
 void pingGSstate()
 {
  rf95.send(PING_STATE_PACKET_GS, 7);
  rf95.waitPacketSent(200);
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
  else if (readyToTrans)
  {
    Serial.println("Transmitting Commands Unlocked");
  }
  else
  {
  if (unlock3)
  {
    Serial.print("Unlocked lock 3");
  }
  if (unlock2)
  {
    Serial.println("Unlocked lock 2");
  }
  if (unlock1)
  {
    Serial.println("Unlocked lock 1");
  }
  if (!(unlock1 || unlock2 || unlock3))
  {
    Serial.println("All locked");
  }
  }
 }
 void pingAll()
 {
  Serial.println("Pinged ALL:");
  pingHCstate();
  pingMCstate();
  pingGSstate();
 }

