from fastapi import FastAPI
from driver import RealMicrocontrollerService, get_logger

import time
from typing import Set
import json

from action_models import ActionRequest

import asyncio
import os
import httpx

## Create a FastAPI instance
app = FastAPI()
# Start the microcontroller service
micro = RealMicrocontrollerService() # Uses the same step model as the StepExample

# Set the namespace for logging
log = get_logger(__name__)
# Set global flags for busy state and stop request
busy = False
stop_requested = False

# --- Helper methods ---
def send_command_to_hardware(pumpA, duration):
    # Extract pump parameters
    stateA = pumpA.state if pumpA else False
    speedA = pumpA.speed if pumpA else 0
    dirA = pumpA.dir if pumpA else True
    
    
    # Send command to hardware
    micro.set_state(
        stateA, speedA, dirA,
        duration
    )
    
    return duration


# Filter pump and duration information from the request and send to hardware
def action_task(request: ActionRequest):
    job_id = request.id
    pumpA = request.pumpA
    duration = request.time
      # Send motor command
    step_time = send_command_to_hardware(pumpA, duration)

# API Endpoints, run http://localhost:8000/docs for quick documentation and testing
@app.post("/actions")
async def perform_actions(request: ActionRequest): # Use the ActionRequest model for request validation
    # Get the global busy and stop_requested flags
    global busy, stop_requested
    if busy:
        # Return a json response indicating that the system is busy
        return {"status": "busy"}
    busy = True
    stop_requested = False
    # Send 'acknowledged' webhook immediately
    
    # If not busy send the action task to the hardware and return an accepted response
    action_task(request)
    return {"status": "accepted", "id": request.id}

@app.post("/stop")
async def emergency_stop(): # Trigger an emergency stop of the pump and reset busy state
    global stop_requested, busy
    micro.stopPumps()
    stop_requested = True
    busy = False  # Fallback: ensure busy is reset if stop is called
    log.info("/stop called: busy set to False")
    return {"status": "stopped"}


@app.get("/status")
def get_status(): # Returns the current busy state of the system
    global busy
    return {"busy": busy} 

