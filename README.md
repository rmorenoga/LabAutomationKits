# LabAutomationKits
Documentation and code for the lab automation teaching kit

## Dev Container (ROS 2 + VNC)
This repo includes a VS Code Dev Container for ROS 2 Humble with desktop + VNC using the base image `tiryoh/ros2-desktop-vnc:humble`.

Quick start:
- Install VS Code and the Dev Containers extension.
- Open this folder in a container (Command Palette: Dev Containers: Reopen in Container).
- Access the GUI via your browser at `http://localhost:6080` (password `vncpassword`).

The user for the container is `ubuntu` with password `ubuntu`. The workspace is mounted at `/workspaces/LabAutomationKits`.

Build and run the ROS 2 workspace inside the container:
```bash
cd /workspaces/LabAutomationKits/ros2_ws
colcon build --packages-select lab_automation_pump_interfaces lab_automation_pump
source install/setup.bash
ros2 launch lab_automation_pump pump.launch.py simulated:=true
```

Send a demo action goal from another terminal:
```bash
ros2 run lab_automation_pump pump_client --duration 3000 --speed 2000 --state 1 --dir 1
```

To use real hardware, pass your serial device ID and ensure the device is mapped in `.devcontainer/devcontainer.json` runArgs.
