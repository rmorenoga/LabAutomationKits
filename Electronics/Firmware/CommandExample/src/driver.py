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
        """Initialize the microcontroller service and establish a connection to the device."""
        log.info("Initializing microcontroller service")
        # Find the COM port for the device
        found, comPort = find_port("0000000-0000-0000-0000-00000000001")

        if found:
            log.info(f"Connected to the device: {comPort}")
        else:
            log.error("No suitable device found.")
            exit()

        self._current_port_id = 1

        # Intialize the board connection with the found COM port and set the baud rate to 115200
        ESP32 = PyCmdMessenger.ArduinoBoard(comPort, baud_rate=115200, timeout=3)
        log.debug(f"Using board: {ESP32}")

        # Define the commands that will be used for communication with the microcontroller, these must match the commands defined in the Arduino sketch
        commands = [["kWatchdog", "s"], # Each command is defined with a name and a format string for the type of arguments it takes
                    ["kAcknowledge", "s"],
                    ["kError", "s"],
                    ["kStart", "?I?"],
                    ["kStop", ""], ]

        # Initialize the Cmdmessenger commander with the board and the defined commands
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
            # Wait for a response from the microcontroller after sending the stop command
            # This will block until a response is received or a timeout occurs
            msg = self.comm.receive()
            # Print the response message for debugging purposes
            log.info(f"Stop pumps response: {msg[1]}")
            return msg[1]
        except EOFError as e:
            log.warning(f"No or incomplete response to stop command: {e}")
            return "No response"



    def start_pumps(self, stateA: bool = False, speedA: int = 0, dirA: bool = True):
        """
        Set the state of pumps A, B, C and the LEDs
        """
        log.info(f"Setting state: A={stateA},{speedA},{dirA} ")
        try:
            # Send the start command to the microcontroller with the specified parameters for pump A
            self.comm.send("kStart", stateA, speedA, dirA)
            # Wait for a response from the microcontroller after sending the start command
            # This will block until a response is received or a timeout occurs
            msg = self.comm.receive()
            log.info(msg)
            log.info(f"State set response: {msg[1]}")
            return True
        except Exception as e:
            log.error(f"Error setting state: {e}")
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
    module = RealMicrocontrollerService()
    module.start_pumps(stateA = True, speedA = 4096, dirA = True)
    
    time.sleep(5)

    module.stopPumps()


if __name__ == "__main__":
    main()