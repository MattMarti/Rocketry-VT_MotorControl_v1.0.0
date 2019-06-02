#include "packet_interpret.h"


    unsigned int  syncBytes[2];

    unsigned int  dataLen;
    unsigned int  ID;

        unsigned int packData[];


    unsigned int checkSumBytes[2];



/*sorts into packet parts and defines namespace variables */
//returns false if not long enough packets
bool sortPacket(unsigned int buf[])
{
    //check that packet is long enough
    if (sizeof(buf) < 6)
    {
        return false;
    }
    unsigned int syncBytes[0] = buf[0];
    unsigned int syncBytes[1] = buf[1];
    unsigned int dataLen = buf[2];
    unsigned int ID = buf[3];
    for (int i = 0; i < dataLen; i++ )
    {
        unsigned int packData[i] = buf[(4 + i)];

    }
    unsigned int checkSumBytes[0] = buf[(4 + dataLen)];
    unsigned int checkSumBytes[1] = buf[(5 + dataLen)];
    return true;
}


/*double check that sync bytes are correct */
bool sanityCheck()
{
    if ((syncBytes[0] == 0xAA) && (syncBytes[1] == 0x14))
    {
        return true
    }
    else
    {
        Serial.println("Packet Interpreting Error");
        Serial.flush();
        return false;
    }

}

/*prints sorted packet */
void printAll()
{
    Serial.print("Sync Bytes: ");
    Serial.println(syncBytes[0], HEX);
    Serial.println(syncBytes[1], HEX);
    Serial.print("Data Length: ");
    Serial.println(dataLen, HEX);
    Serial.print("ID: ");
    Serial.println(ID, HEX);
    Serial.print("Data: ");
    if (dataLen > 0)
    {


    for (int i = 0; i < dataLen; i++ )
    {
        Serial.println(packData[i], HEX);

    }
    }
    else{
        Serial.println("Empty");
    }
    Serial.print("Checksum Bytes: ");
    Serial.println(checkSumBytes[0], HEX);
    Serial.println(checkSumBytes[1], HEX);
}
