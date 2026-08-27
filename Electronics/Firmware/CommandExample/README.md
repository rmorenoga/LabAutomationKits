# CommandExample — CmdMessenger Firmware Example

This repository contains a firmware example demonstrating use of the CmdMessenger library for serial command-based communication between a host (PC) and an Arduino embedded device.

Key points
- Uses PlatformIO for building and uploading firmware.
- Includes Python helper scripts for finding and talking to the device over serial.
- The core example firmware and support libraries live under `src/` and `lib/` respectively.
- Check the correspondance between driver.py and main.cpp these two contain the main parts of the communication protocol

Requirements
- PlatformIO (VS Code extension or `platformio` CLI)
- Python 3.8+ (for helper scripts)
- `pip` and the Python dependencies listed in `requirements.txt` (e.g. `pyserial`).

Quick setup

1. Prepare Python environment (Windows PowerShell example):

```powershell
python -m venv .venv
& .venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

2. Build and upload firmware with PlatformIO (from repo root):

Use the control buttons at the bottom of VSCode or run the following commands in the terminal

```bash
# build
platformio run
# build and upload (choose your environment in platformio.ini)
platformio run --target upload
```

3. Monitor serial from PlatformIO:

```bash
platformio device monitor
```

Using the Python helper scripts

- `src/find_port.py` — simple helper to detect likely serial ports for the board.
- `src/driver.py` — example host-side driver that opens a serial connection and sends/receives commands compatible with the firmware example.

Typical workflow

1. Build/upload firmware to your board with PlatformIO.
2. Run the Python driver to exercise commands and observe responses:

```bash
# (with venv activated)
python src/driver.py
```

Project structure (high level)

- `platformio.ini` — PlatformIO project configuration.
- `src/` — host helper scripts (`driver.py`, `find_port.py`) and `main.cpp` for firmware.
- `lib/CmdMessenger/` — CmdMessenger library and example sketches.
- `dataModel/` — library for storing the current state of the pumps
- `DF4MotorDriver/` — library with definitions and functions to interface the motor driver 
- `requirements.txt` — Python dependencies for helper scripts.

