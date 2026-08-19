# PumpExample Firmware

Small PlatformIO-based firmware example for a pump controller. This repository demonstrates a compact firmware layout with a simple data model and a motor driver library.

## Requirements

- PlatformIO CLI or VS Code PlatformIO extension
- Compatible embedded toolchain defined by `platformio.ini`

## Repository structure

- `platformio.ini` - Project configuration, board, framework and build options.
- `include/` - Public header files used by the project (project-wide includes).
- `lib/` - Local libraries used by the firmware:
  - `dataModel/` - Implements the `dataModel` used to track pump settings and state (`dataModel.cpp`, `dataModel.h`).
  - `DF4MotorDriver/` - DF4 motor driver abstraction (`df4MotorDriver.cpp`, `df4MotorDriver.h`).
- `src/` - Application sources:
  - `main.cpp` - Program entry point and application logic wiring the data model and motor driver.


## Build & Upload

Use PlatformIO commands from the repository root:

```
pio run                  # build
pio run -t upload        # build and upload to the configured board
pio device monitor       # open serial monitor (if applicable)
```

If using VS Code, the PlatformIO extension provides GUI commands for building and uploading.

## Notes for contributors

- Keep hardware-specific code isolated in `lib/DF4MotorDriver` so it can be swapped or mocked for tests.
- Keep the data/state logic inside `lib/dataModel` to make `main.cpp` simpler and easier to test.

