#ifndef DATAMODEL_H_
#define DATAMODEL_H_


#include <Arduino.h>


// Class to represent the state of a pump, including its on/off state, speed, and direction
class Pump{
    public:
        bool state = false;
        uint16_t speed = 0;
        bool dir = true;
};


extern Pump pumpA;
extern Pump pumpB;
extern Pump pumpC;


#endif