#include "../RC_stuff/rc_pilot_reading.h"
#include "../RC_stuff/rc_pilot.h"
#include "Servo.h"
#include "Arduino.h"

extern RC_PILOT rc;
int16_t throttle = 0;

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int esc1Pin = 9;
int esc2Pin = 10;
int esc3Pin = 11;
int esc4Pin = 12;

int16_t throttle_control(int16_t throttle_controller) {
    int16_t input_min = 1000;
    int16_t input_max = 2000;
    int16_t output_min = 1000;
    int16_t output_max = 1250;

    // Linear interpolation formula
    int16_t throttle_desired = output_min + (throttle_controller - input_min) * (output_max - output_min) / (input_max - input_min);
    return throttle_desired;
}

void controller_setup() {
    Serial.begin(115200);
    esc1.attach(esc1Pin);
    esc2.attach(esc2Pin);
    esc3.attach(esc3Pin);
    esc4.attach(esc4Pin);

    delay(1000);
}

void controller_loop() {
    throttle = throttle_control(rc.rc_in.THR);
    esc1.writeMicroseconds(throttle);
    esc2.writeMicroseconds(throttle);
    esc3.writeMicroseconds(throttle);
    esc4.writeMicroseconds(throttle);
    Serial.print(throttle);
}