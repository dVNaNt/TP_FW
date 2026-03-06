#include <Arduino.h>

#include "tunepulse.h"


void setup() {
    delay(2000);

    SerialUSB.begin();
    tunepulse_init();

    SerialUSB.println("Going LOOP");
}

void loop() {
    TUNE_HEART.tick();
}
