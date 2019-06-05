/*test MC hardware dick about yo */
#include <motor_controller_config.h>;
#include <MC_Hardware.h>;
using namespace MC_Pins;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  uint8_t cont[3] = {0, 0, 0};
  checkCont(cont);
  Serial.println(cont[0], DEC);
  pinMode(FLOAT_SWITCH_PIN, INPUT_PULLUP);
  //digitalWrite(FLOAT_SWITCH_PIN, HIGH);
  delay(1000);
  //crackVent();
  //openVentFull();
  //delay(100);
  //pinsOff();
  //delay(500);
  digitalWrite(VENT_DIG_PIN1, LOW);
        digitalWrite(VENT_DIG_PIN2, LOW);
        
        //closeOxValve();
       // delay(400);
        pinsOff();
        //closeOxValve();
        delay(500);
        fireEmatch();
        delay(5000);
        pinsOff();
  

}

void loop() {
  // put your main code here, to run repeatedly:
    uint8_t cont[3] = {0, 0, 0};
  checkCont(cont);
  delay(100);
  if (cont[0] == 1)
  {
  Serial.println("cont on vent");
  }
  delay(300);
  if (cont[1] == 1)
  {
   Serial.println("cont ox");
  }
   if (cont[2] == 1)
  {
   Serial.println("ematch continuity");
  }
    delay(300);
  bool tank = tankFull();
 
  if (tank)
  {
    Serial.println("full");
  }
  delay(300);
}
