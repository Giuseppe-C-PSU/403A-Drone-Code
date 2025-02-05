
#include "src/RC_stuff/rc_pilot_reading.h"
#include "src/sensor_stuff/sensor_prelim.h"
#include "src/sensor_stuff/sensors.h"
#include "src/EKF_stuff/EKF.h"
EKF ekf;
extern Sensors sens;


void setup()
{

 rc_setup();

 pozyx_setup();

 thermal_setup();


}

unsigned long time = 0;
void loop()
{
  time = millis();
  //Serial.println(time);


  rc_reciever_loop();
  //delay(100);

  Serial.print("Pozyx: ");
  pozyx_loop();
  //delay(100);

  ekf.predict();

  // Example measurement (3 measurements)
  float z[3] = {sens.data.euler[0], sens.data.euler[1], sens.data.euler[2] };

  // Update step with the measurement
  ekf.update(z);

  // Output the current state estimate after the update
  Serial.print("Estimated State: ");
  ekf.printState();

  if (time%1000 == 0){
    Serial.println();
    Serial.print("Thermal Camera\n");
    thermal_loop();
    Serial.print("\n");
  }
  
  //delay(1000);

  
}