#include <Arduino.h>
#include "dataModel.h"
#include "df4MotorDriver.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");
  setupPumps();
  Serial.println("Setup finished");
}

void loop() {
  delay(3000);
  Serial.println("Stopping pumps");
  StartPumpA(4096, HIGH);

  Serial.println("Starting pump A forward at half speed for 3 seconds");
  delay(3000);
  StartPumpA(2048, HIGH);

  delay(3000);
  stopPumps();
  Serial.println("Pumps stopped");



}