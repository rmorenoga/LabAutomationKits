#include <Arduino.h>
#include <CmdMessenger.h>
#include "dataModel.h"
#include "df4MotorDriver.h"

// Create a CmdMessenger object, passing in the hardware serial port and the command separators
CmdMessenger cmdMessenger(Serial, ',', ';', '/');

// This is the list of recognized commands. These can be commands that can either be sent or received.
// In order to receive, attach a callback function to these events
void OnUnknownCommand();
void OnWatchdogRequest();
void OnArduinoReady();
void OnReceiveStart();
void OnReceiveStop();

void receiveStart();
void receiveStop();

// Command IDs for the commands we send from the Arduino and want to receive on the PC.
enum
{
  // Commands
  kWatchdog,          // Command to request application ID
  kAcknowledge,       // Command to acknowledge a received command
  kError,             // Command to message that an error has occurred
  kStart,              // Command to receive a step (pump state + time), should always contain the full state of the pumps
  kStop,              // Command to stop all pumps

};

// Commands we send from the PC and want to receive on the Arduino.
// We must define a callback function in our Arduino program for each entry in the list below.
void attachCommandCallbacks()
{
  // Attach callback methods to commands we expect to receive from the PC
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kWatchdog, OnWatchdogRequest);
  cmdMessenger.attach(kStart, OnReceiveStart);
  cmdMessenger.attach(kStop, OnReceiveStop);
}

// ------------------  C A L L B A C K S -----------------------

// Called when a received command has no attached function
void OnUnknownCommand()
{
  cmdMessenger.sendCmd(kError, "Command without attached callback");
}
// Called when a received command is a watchdog request
void OnWatchdogRequest()
{
  // Will respond with same command ID and Unique device identifier.
  cmdMessenger.sendCmd(kWatchdog, "0000000-0000-0000-0000-00000000001");
}

// Callback function that responds that Arduino is ready (has booted up)
void OnArduinoReady()
{
  cmdMessenger.sendCmd(kAcknowledge, "Arduino ready");
}

// Callback function that responds to a received command to start the pump
void OnReceiveStart()
{
  receiveStart();
}

// Callback function that responds to a received command to stop the pump
void OnReceiveStop()
{
  receiveStop();
}



void setup() {
  // Start the serial port
  Serial.begin(115200);

  // Setup the IO pins for the pumps and initialize the pump state
  setupPumps();

  //  Do not print newLine at end of command,
  //  in order to reduce data being sent
  cmdMessenger.printLfCr(false);

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();
  
  // Send a command to the PC to indicate that the Arduino has started
  cmdMessenger.sendCmd(kAcknowledge, "Arduino has started!");
}

void loop() {

  // Wait for a command to arrive, and call the appropriate callback function when it does.
  cmdMessenger.feedinSerialData();

}

// ------------------  C O M M A N D   H A N D L E R S -----------------------
// Callback function that responds to a received command to start the pump
void receiveStart()
{
    // Read incoming arguments one by one, in this case a bool an integer and a bool, which represent the state, speed and direction of the pump
    pumpA.state = cmdMessenger.readBinArg<bool>();
    pumpA.speed = cmdMessenger.readBinArg<uint16_t>();
    pumpA.dir = cmdMessenger.readBinArg<bool>();

    // If the pump state is true, start the pump with the specified speed and direction, otherwise stop the pump
    if (pumpA.state)
    {
      StartPumpA(pumpA.speed, pumpA.dir);
    }
    else
    {
      StopPumpA();
    }

    // Send an acknowledgment command back to the PC with a message indicating that the pump has started
   cmdMessenger.sendCmd(kAcknowledge, "Pump A started!");
}

// Callback function that responds to a received command to stop the pump
void receiveStop()
{
  // Stop all pumps by setting the PWM to 0 and updating the pump state in the data model
  stopPumps();
  
  // Send an acknowledgment command back to the PC with a message indicating that all pumps have been stopped
  cmdMessenger.sendCmd(kAcknowledge, "Stopped");
}