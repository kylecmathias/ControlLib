#ifndef CLBSTARTUP_H
#define CLBSTARTUP_H

#include <Arduino.h>

namespace clb {
    static void startup() {
        Serial.begin(9600);
        delay(1000); 
        Serial.println("Starting Arduino with serial baud rate 9600");
        
    }
}

#endif