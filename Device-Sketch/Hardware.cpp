#include "Hardware.h"
#include "Default_Config.h"
#include "XBee_IO.h"
#include "Transmission.h"
#include "Arduino.h"
#include "Adafruit_MAX31855.h"
#include "HX711.h"

/* Load Cell object */
HX711 loadcell(Pins_Config::LOADCELL_DOUT, Pins_Config::LOADCELL_CLK);

/*intialize thermocouples*/
Adafruit_MAX31855 thermocouple_1(Pins_Config::pin_T1_CLK, Pins_Config::pin_T1_CS, Pins_Config::pin_T1_DO);
Adafruit_MAX31855 thermocouple_2(Pins_Config::pin_T2_CLK, Pins_Config::pin_T2_CS, Pins_Config::pin_T2_DO);
Adafruit_MAX31855 thermocouple_3(Pins_Config::pin_T3_CLK, Pins_Config::pin_T3_CS, Pins_Config::pin_T3_DO);

//create motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
/*connect stepper with 200 steps per rotation*/
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);

/* LED Data */
bool Hardware::is_LED_on = false;

/**
   Updates data variables by calling functions that control data
   acquisition from connected devices

   INPUT
   void

   OUTPUT
   Hexidecimal value, one activated for each datum obtained successfully
          0x00000001 - PRESSURE_OXIDIZER
          0x00000010 - PRESSURE_COMBUSTION
          0x00000100 - TEMPERATURE_OXIDIZER
          0x00001000 - TEMPERATURE_COMBUSTION
          0x00010000 - THRUST
*/
void Hardware::update_data(unsigned long time) {
  
	using namespace State_Data;
	using namespace Default_Config;
	
	// New Data
	unsigned char nd = 0;
	DATA_TIME = time;
	
	// Update Pressure
	if (time - LAST_PRESSURE_TIME_US > PRESSURE_PERIOD_MS*1000) {
		//DATA_P1 = get_pressure_1_data(); // Insert Patrick's code here
		//DATA_P2 = get_pressure_2_data(); // Insert Patrick's code here
		LAST_PRESSURE_TIME_US = time;
		nd |= 0x01;
	}
	
	// Update Temperature
	if (time - LAST_TEMPERATURE_TIME_US > TEMPERATURE_PERIOD_MS*1000) {

		XBee.end();
		SDCard.end();
		DATA_T1 = thermocouple_1.readFarenheit();
		//DATA_T2 = thermocouple_2.readFarenheit();
		//DATA_T3 = thermocouple_3.readFarenheit();
		XBee.begin(Default_Config::XBEE_BAUD);
		SDCard.begin(Default_Config::SD_BAUD);
		
		LAST_TEMPERATURE_TIME_US = time;
		nd |= 0x02;
	}
	
	// Update Load Cell
	if (time - LAST_LOADCELL_TIME_US > LOADCELL_PERIOD_MS*1000) {
		DATA_THR = loadcell.get_units(); // Load cell measure thrust
		LAST_LOADCELL_TIME_US = time;
		nd |= 0x04;
	}
	
	NEW_DATA = nd;
	return;
}

/**
   Creates a file to save data to
*/
void Hardware::sdcard_openfile() {
	//SDCard.begin(Default_Config::SD_BAUD); // .doing .begin(baud) will not make a new file
}

/**
   Closes currently open save file
*/
void Hardware::sdcard_closefile() {
	//SDCard.end(); // .doing .begin(baud) will not make a new file
}

/**
   Saves the data values to a line on a file in the SD card
*/
void Hardware::sdcard_write(unsigned int datatype) {
	unsigned int len = 0;
	Transmission::buildPacket(XBeeIO::output_buff, &len, datatype);
	SDCard.write(XBeeIO::output_buff, len);
}

/**
   Initializes the stepper motor
*/
void Hardware::initializeStepperMotor() {
  AFMS.begin(); //create default freq of 1.6khz
  myMotor->setSpeed(100);  // set to 100 rpm
}

/**
   Opens the stepper motor
*/
void Hardware::openStepperMotor() {
  myMotor->step(560, FORWARD, SINGLE); // number of steps to fully open valve
}
/**
   Closes the stepper motor
*/
void Hardware::closeStepperMotor() {
  myMotor->step(560, BACKWARD, SINGLE); //close valve
}

/**
	Sets the calibration for the Load Cell
*/
void Hardware::initializeLoadCell() {
	loadcell.set_scale(Pins_Config::loadcell_calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
	loadcell.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
}

/**
   reads pressure in oxidizer tank
   @return float = pressure in psi
*/
float Hardware::get_pressure_1_data()
{
  int sensorVal = analogRead(Pins_Config::pin_P1);
  float voltage = (sensorVal * 5.0) / 1024.0;
  float pressure_psi = (((250.0f * voltage)) - 125.0f);
  return pressure_psi;
}

/**
   reads pressure in cobustion chamber
   @return float = pressure in psi
*/
float Hardware::get_pressure_2_data()
{
  int sensorVal = analogRead(Pins_Config::pin_P2);
  float voltage = (sensorVal * 5.0) / 1024.0;
  float pressure_psi = (((250.0f * voltage)) - 125.0f);
  return pressure_psi;
}

/**
   Turns on the LED

   INPUT
   bool output -> true to output "LED ON", false by default
*/
void Hardware::turn_LED_on(bool output) {
  if (output) XBee.print("LED ON\n");
  is_LED_on = true;
  digitalWrite(Pins_Config::pin_LED, HIGH); // TOGGLE LED ON
}

/**
   Turns off the LED

   INPUT
   bool output -> true to output "LED OFF", false by default
*/
void Hardware::turn_LED_off(bool output) {
  if (output) XBee.print("LED OFF\n");
  is_LED_on = false;
  digitalWrite(Pins_Config::pin_LED, LOW); // TOGGLE LED ON
}

