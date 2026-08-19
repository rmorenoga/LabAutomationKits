#include "df4MotorDriver.h"

// Function to start Pump A with a specified speed and direction
void StartPumpA(uint16_t speed, int dir){
    //Setting initial velocity(Min:0  Max:127)
    // Use the map function to scale the speed from a range of 0-4096 to 0-4096 for the PWM output
    uint16_t speedCorrected = map(speed,0,4096,0,4096);
    // Set the direction pin for Pump A based on the provided direction value
    digitalWrite(DFDRIVER_DIR_1,dir);
    // Set the PWM output for Pump A to control its speed
    analogWrite(DFDRIVER_PWM_1,speedCorrected);
}

void StartPumpB(uint16_t speed, int dir){
    //Setting initial velocity(Min:0  Max:127)
    uint16_t speedCorrected = map(speed,0,4096,0,4096);
    digitalWrite(DFDRIVER_DIR_2,dir);
    analogWrite(DFDRIVER_PWM_2,speedCorrected);
}

void StartPumpC(uint16_t speed, int dir){
    //Setting initial velocity(Min:0  Max:127)
    uint16_t speedCorrected = map(speed,0,4096,0,4096);
    digitalWrite(DFDRIVER_DIR_3,dir);
    analogWrite(DFDRIVER_PWM_3,speedCorrected);
}

// Function to stop Pump A by setting its PWM output to 0
void StopPumpA(){
   analogWrite(DFDRIVER_PWM_1,0);   
}

void StopPumpB(){
   analogWrite(DFDRIVER_PWM_2,0);   
}

void StopPumpC(){
   analogWrite(DFDRIVER_PWM_3,0);   
}

// Function to set up the pumps by configuring the PWM resolution and setting the pin modes for the PWM and direction pins
void setupPumps(){
    analogWriteResolution(12);
    pinMode(DFDRIVER_PWM_1,OUTPUT);
    pinMode(DFDRIVER_PWM_2,OUTPUT);
    pinMode(DFDRIVER_PWM_3,OUTPUT);
    pinMode(DFDRIVER_PWM_4,OUTPUT);
    pinMode(DFDRIVER_DIR_1,OUTPUT);
    pinMode(DFDRIVER_DIR_2,OUTPUT);
    pinMode(DFDRIVER_DIR_3,OUTPUT);
    pinMode(DFDRIVER_DIR_4,OUTPUT);
}

// Function to stop all pumps by setting their PWM outputs to 0 and updating their states in the data model
void stopPumps(){
  pumpA.state = false;
  StopPumpA();
  pumpB.state = false;
  StopPumpB();
  pumpC.state = false;
  StopPumpC();
}

// Global object to hold the current step in the pump control sequence
Step currentStep;

