/*test MC hardware dick about yo */
#include "motor_controller_config.h";
#include "MC_Hardware.h";
using namespace MC_Pins;
Encoder encoder(CH_A, CH_B);  // Definition of incrementation encoder
void setup() {
  
  Serial.begin(115200);
 //Serial2.begin(115200);
  Serial2.println(5.0);
  // put your setup code here, to run once:
  uint8_t cont[3] = {0, 0, 0};
  checkCont(cont);
  Serial.println(cont[0], DEC);
  pinMode(FLOAT_SWITCH_PIN, INPUT_PULLUP);
  //digitalWrite(FLOAT_SWITCH_PIN, HIGH);
 // delay(1000);
  //crackVent();
  //openVentFull();
  //delay(100);
  //pinsOff();
  //delay(500);
  digitalWrite(VENT_DIG_PIN1, LOW);
        digitalWrite(VENT_DIG_PIN2, LOW);
        
        //openOxValve();
       //delay(400);
        pinsOff();
       //delay(400);
        //openOxValve();
       // delay(500);
        fireEmatch();
      //  delay(5000);
        pinsOff();
  

}

void loop() {
 //Serial2.print(5);
  //Serial.println(readOxPressure(), DEC);
  //delay(1000);
  //uint8_t tempInj = readInjTemp();
 // delay(1000);
  //Serial.println("returned inj temp");
  //Serial.println((tempInj*10), DEC);
  //delay(1000);

   //uint8_t tempNoz = readNozTemp();
  //delay(1000);
 // Serial.println("returned noz temp");
  //Serial.println((tempNoz*10), DEC);
  // put your main code here, to run repeatedly:
   // uint8_t cont[3] = {0, 0, 0};
 // checkCont(cont);
 // delay(100);
//  if (cont[0] == 1)
//  {
//  Serial.println("cont on vent");
//  }
//  delay(300);
//  if (cont[1] == 1)
//  {
//   Serial.println("cont ox");
//  }
//   if (cont[2] == 1)
//  {
//   Serial.println("ematch continuity");
//  }
//    delay(300);
  bool tank = tankFull();
// 



  
  uint8_t pos = encoder.read();


  double raw_degree = pos * (360.0 / 4096.0)*(4.0/3.0);
  if (raw_degree < 0)
  {
    return 360.0 + raw_degree;
  }
  raw_degree = (uint8_t) raw_degree;


Serial.println(raw_degree);
crackVent();
delay(2000);
  if (tank)
  {
    Serial.println("full");
  }
//  delay(300);
}
