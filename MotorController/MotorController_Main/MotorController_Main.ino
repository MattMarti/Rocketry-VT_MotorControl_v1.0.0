#include <CircularBuffer.h>
#include <SPI.h>
#include <RH_RF95.h>

const uint8_t RFM95_CS = 4;
const uint8_t RFM95_RST = 2;
const uint8_t RFM95_INT = 3;
const double RF95_FREQ = 434.0;
const size_t BUFFER_SIZE = RH_RF95_MAX_MESSAGE_LEN * 2;
const uint8_t REPLY_PACKET[14] = {0xAA, 0x14, 0x01, 0x69, 0x04, 0xF3, 0x71, 0xAA, 0x14, 0x01, 0x69, 0x09, 0xF3, 0x71};
const uint32_t serial_baudrate = 38400;

RH_RF95 rf95(RFM95_CS, RFM95_INT);

CircularBuffer<uint8_t, BUFFER_SIZE> buf4;

/**
    Parses the provided buffer for packets, and removes all bytes which were processed.
*/
CircularBuffer<uint8_t, BUFFER_SIZE> parse_packet(CircularBuffer<uint8_t, BUFFER_SIZE>& buf);

/**
    Listens for a short time and stores all data from the radio,
    up to the maximum size of the buffer.
*/
void radio_recieve(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer);

/**
    Writes a data buffer to 3 serial ports, and sends a confirmation packet
    over the radio.
*/
void echo_radio(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer,
                HardwareSerial &dest1, HardwareSerial &dest2, HardwareSerial &dest3);

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(serial_baudrate);

    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init())
    {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    Serial.println("LoRa radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ))
    {
        Serial.println("setFrequency failed");
        while (1);
    }

    Serial.print("Set Freq to: ");
    Serial.println(RF95_FREQ);
    rf95.setTxPower(23, false);

}

void loop() {

    radio_recieve(buf4);

    CircularBuffer<uint8_t, BUFFER_SIZE> fromRadio = parse_packet(buf4);
    

    // echoes, and sends confirmation to, the radio
    echo_radio(fromRadio);
}

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
                Serial.print(buf[i], HEX);
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


void echo_radio(CircularBuffer<uint8_t, BUFFER_SIZE> &buffer) 
{
    uint8_t toSend[buffer.size()];
    const size_t bufferSize = buffer.size();
    for (size_t i = 0; buffer.size() > 0; i++)
    {
        toSend[i] = buffer.shift();
    }

    if (bufferSize > 0)
    {
      uint8_t x[7] = {0xAA, 0x14, 0x01, 0xF5, 0x35, 0xBF, 0x91};
        rf95.send(x, 7);
    }
}
