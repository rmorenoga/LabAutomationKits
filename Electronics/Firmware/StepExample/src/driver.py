import os
import PyCmdMessenger
import time
import logging
from find_port import find_port

def get_logger(name):
    logging.basicConfig(level=logging.DEBUG)
    return logging.getLogger(name)

log = get_logger(__name__)

class RealMicrocontrollerService:
    """
    The actual microcontroller service for connecting to hardware.
    """

    def __init__(self):
        log.info("Initializing microcontroller service")
        found, comPort = find_port(os.getenv("DEVICE_ID"))

        if found:
            log.info(f"Connected to the device: {comPort}")
        else:
            log.error("No suitable device found.")
            exit()

        self._current_port_id = 1

        ESP32 = PyCmdMessenger.ArduinoBoard(comPort, baud_rate=115200, timeout=3)
        log.debug(f"Using board: {ESP32}")

        commands = [["kWatchdog", "s"],
                    ["kAcknowledge", "s"],
                    ["kError", "s"],
                    ["kGetState", ""],
                    ["kGetStateResult", "?I??I??I??I?I?I?I?I"],
                    ["kGetLastStep", ""],
                    ["kGetLastStepResult", "??L?I??I??I??I?I?I?I?I"],
                    ["kStep", "?I??I??I??I?I?I?I?IL"],
                    ["kStop", ""],
                    ["kStepDone", ""], ]

        # Initialize the messenger
        self.comm = PyCmdMessenger.CmdMessenger(ESP32, commands)
        log.info("Messenger initialized")
        # Wait for arduino to come up
        msg = self.comm.receive()
        log.info(f"Initial communication: {msg}")

    def stopPumps(self):
        """Function for stopping all pumps."""
        log.info("Sending stop command to all pumps")
        self.comm.send("kStop")
        try:
            msg = self.comm.receive()
            log.info(f"Stop pumps response: {msg[1]}")
            return msg[1]
        except EOFError as e:
            log.warning(f"No or incomplete response to stop command: {e}")
            return "No response"


    def getState(self):
        """Get the current state of the microcontroller."""
        log.info("Getting microcontroller state")
        result = []
        self.comm.send("kGetState")

        # Receive state data
        msg = self.comm.receive()

        result = msg[1]
        log.info(f"Current state: {result}")

        return result

    def get_state_pretty(self):
        """
        Pretty wrapper for getState
        
        Returns state in a processed format:
        - If state is a list: converts to dictionary with pumpA and pumpB keys
        - Otherwise returns the raw state
        """
        raw_state = self.getState()
        
        # If state is a list in the format [stateA, speedA, dirA, stateB, speedB, dirB]
        if isinstance(raw_state, list) and len(raw_state) >= 3:
            # Convert to dictionary format for easier processing
            return {
                "pumpA": {
                    "state": bool(raw_state[0]),
                    "speed": int(raw_state[1]),
                    "dir": bool(raw_state[2])
                },
                
            }
        # Return raw state if not in expected list format
        return raw_state

    def getLastStep(self):
        """Get the last pump step command sent to the microcontroller."""
        log.info("Getting last step information")
        result = []
        self.comm.send("kGetLastStep")

        # Receive state data
        msg = self.comm.receive()

        result = msg[1]
        log.info(f"Last step: {result}")

        return result


    def set_state(self, stateA: bool = False, speedA: int = 0, dirA: bool = True,
                       stateB: bool = False, speedB: int = 0, dirB: bool = True,
                       stateC: bool = False, speedC: int = 0, dirC: bool = True, 
                       stateLed0: bool = False, intensityLed0: int = 0,
                       stateLed1: bool = False, intensityLed1: int = 0,
                       stateLed2: bool = False, intensityLed2: int = 0,
                       stateLed3: bool = False, intensityLed3: int = 0,
                       stateLed4: bool = False, intensityLed4: int = 0, stepTime: int = 50000):
        """
        Set the state of pumps A, B, C and the LEDs
        """
        log.info(f"Setting state: A={stateA},{speedA},{dirA} B={stateB},{speedB},{dirB} C={stateC},{speedC},{dirC} led0={stateLed0},{intensityLed0} led1={stateLed1},{intensityLed1} led2={stateLed2},{intensityLed2} led3={stateLed3},{intensityLed3} led4={stateLed4},{intensityLed4} time={stepTime}")
        try:
            self.comm.send("kStep", stateA, speedA, dirA, stateB, speedB, dirB, stateC, speedC, dirC, stateLed0, intensityLed0, stateLed1, intensityLed1, stateLed2, intensityLed2, stateLed3, intensityLed3, stateLed4, intensityLed4, stepTime)
            msg = self.comm.receive()
            log.info(msg)
            log.info(f"State set response: {msg[1]}")
            return True
        except Exception as e:
            log.error(f"Error setting state: {e}")
            return False

    
    def check_for_step_done(self) -> bool:
        """Check if the current step operation has completed."""
        log.debug("Checking for step completion")
        try:
            msg = self.comm.receive()
            if msg is not None:
                log.debug(f"Step check message: {msg[0]}")
            if msg is not None and msg[0] == "kStepDone":
                log.info("Step operation completed")
                current_state = self.getState()
                log.info(f"Fetched state after stop command: {current_state}")
                return True
        except EOFError as e:
            log.warning(f"Incomplete message when checking for step done: {e}")
        return False

    def close(self):
        """
        Closes the serial connection.
        """
        try:
            self.ser.close()
            log.info("Serial connection closed")
        except Exception as e:
            log.error(f"Error closing connection: {str(e)}")



def main():
    """Main function for testing the service directly."""
    tile = RealMicrocontrollerService()
    tile.set_state(stateA = True, speedA = 4096, dirA = True, stepTime = 50000)
    
    tile.getLastStep()
    while not tile.check_for_step_done():
        log.info("Waiting for step to complete...")
        time.sleep(1)


if __name__ == "__main__":
    main()