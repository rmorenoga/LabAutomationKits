from fastapi import FastAPI, BackgroundTasks, WebSocket, WebSocketDisconnect, HTTPException
from driver import RealMicrocontrollerService, get_logger

import time
from typing import Set
import json

from action_models import ActionRequest

import asyncio
import os
import httpx

app = FastAPI()
micro = RealMicrocontrollerService()

log = get_logger(__name__)
busy = False
stop_requested = False

main_event_loop = None

@app.on_event("startup")
async def startup_event():
    global main_event_loop
    main_event_loop = asyncio.get_running_loop()

@app.post("/actions")
async def perform_actions(request: ActionRequest, background_tasks: BackgroundTasks):
    global busy, stop_requested
    if busy:
        return {"status": "busy"}
    busy = True
    stop_requested = False
    # Send 'acknowledged' webhook immediately
    
    background_tasks.add_task(action_task, request)
    return {"status": "accepted", "id": request.id}