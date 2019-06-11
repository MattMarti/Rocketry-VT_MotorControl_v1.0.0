#include <SPI.h>
#include <SD.h>
#include <HX711.h>

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 8;

// HX711 circuit wiring
const uint8_t LOADCELL_DOUT_PIN = 4;
const uint8_t LOADCELL_SCK_PIN = 2;
HX711 scale;
const double calibration_factor = -2100;

unsigned long beginTime;
File logfile;

void setup()
{
    Serial.begin(115200);

    Serial.println("Setting up HX711...");
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale();
    scale.tare(); // Reset the scale to 0
    scale.set_scale(calibration_factor);

    Serial.println("Initializing SD card...");
    pinMode(10, OUTPUT);
    pinMode(chipSelect, OUTPUT);

    if (!SD.begin(chipSelect))
    {
        while(1)
        {
            Serial.println("Card failed, or not present");
            delay(1000);
        }
    }

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    beginTime = millis();
    Serial.println("All good!");

    logfile = SD.open("loadcell.dat", FILE_WRITE);
    logfile.println("======================================");
}

void loop()
{    
    if (scale.is_ready())
    {
        double lbf = scale.get_units();
        unsigned long nowTime = millis() - beginTime;
        logfile.println(String(nowTime) + " " + String(lbf));
        logfile.flush();
        Serial.println(lbf);
    }
}

