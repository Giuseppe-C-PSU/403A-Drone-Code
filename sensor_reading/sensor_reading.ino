// copyright Giuseppe Adamo fucking Carbone!
// Bitch

#include "sensor_prelim.h"
#include "sensors.h"


void setup() {
  pozyx_setup();
 
  delay(1000);
  thermal_setup();
  
  delay(5000);
}

void loop() {
  Serial.print("Pozyx\n");
  pozyx_loop();
  Serial.print("\n");
  delay(1000);

  Serial.print("Thermal Camera\n");
  thermal_loop();
  Serial.print("\n");
  delay(1000);
}


