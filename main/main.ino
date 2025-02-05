
#include "src/RC_stuff/rc_pilot_reading.h"
#include "src/sensor_stuff/sensor_prelim.h"
#include "src/sensor_stuff/sensors.h"


void setup()
{

    rc_setup();

    pozyx_setup();

    // thermal_setup();


}


void loop()
{

    rc_reciever_loop();
    // delay(1000);

    Serial.print("Pozyx\n");
    pozyx_loop();
    Serial.print("\n");
    // delay(1000);


    // Serial.print("Thermal Camera\n");
    // thermal_loop();
    // Serial.print("\n");
    // delay(1000);
}