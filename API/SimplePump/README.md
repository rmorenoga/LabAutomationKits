# SimplePump API

SimplePump is a small FastAPI service for sending pump step commands to a connected microcontroller.

## How It Works

The application starts in `main.py`. It creates a FastAPI application and initializes `RealMicrocontrollerService` from `driver.py`. The driver discovers the microcontroller's serial port and communicates with it using `PyCmdMessenger`.

The hardware communication reuses the `Step` model from `StepExample`, allowing the same step-based command format to be used when communicating with the hardware.

Incoming requests are validated with the Pydantic models in `action_models.py`:

- `ActionRequest` contains an action ID, a duration in milliseconds, and an optional `pumpA` command.
- `PumpCommand` contains the pump state, speed, and direction.

When an action is accepted, `main.py` passes the pump settings and duration to the microcontroller as a step command. The service currently tracks one action at a time with a global busy flag.

## Requirements

- Python 3.9 or newer
- A supported microcontroller connected over USB/serial
- The device identifier expected by `find_port.py`

Install the Python dependencies with:

```powershell
pip install -r requirements.txt
```

## Run the API

From this directory, start the service with:

```powershell
uvicorn main:app --reload
```

The API is available at `http://localhost:8000`. FastAPI's interactive documentation is available at `http://localhost:8000/docs`.

The microcontroller is initialized when the application starts. If no suitable serial device is found, startup exits.

## Endpoints

### `POST /actions`

Starts a pump action. If another action is running, the API returns `busy`.

Example request:

```json
{
  "id": "test-1",
  "time": 10000,
  "pumpA": {
    "state": true,
    "speed": 2000,
    "dir": true
  }
}
```

A successful request returns:

```json
{
  "status": "accepted",
  "id": "test-1"
}
```

`time` is the step duration in milliseconds. If `pumpA` is omitted, the pump defaults to stopped with speed `0`.

### `POST /stop`

Sends a stop command to all pumps, marks the current action as stopped, and clears the busy state.

```json
{
  "status": "stopped"
}
```

### `GET /status`

Returns whether the service is currently handling an action.

```json
{
  "busy": false
}
```

## Project Files

- `main.py` - FastAPI application and API endpoints
- `action_models.py` - Request validation models
- `driver.py` - Serial communication and microcontroller commands
- `find_port.py` - Serial port discovery
- `requirements.txt` - Python dependencies
