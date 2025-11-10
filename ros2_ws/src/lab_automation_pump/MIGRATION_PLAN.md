# Migration & Lecture Plan

## Goal
Transform the existing SimplePumpBackground (FastAPI + background task) into a ROS 2 package demonstrating actions, topics, and hardware abstraction.

## Phases
1. Inspect legacy code (driver + main.py).
2. Identify timed operation -> ROS 2 Action (PumpStep).
3. Define messages & action file.
4. Implement mock driver first (no hardware dependency).
5. Build action server + state publisher.
6. Demonstrate goal execution, feedback, cancel.
7. Swap to real hardware driver.
8. Extension exercises.

## Exercises
- Add stop service using std_srvs/Trigger.
- Add parameter to change publish rate live (on-set callback).
- Add second pump channel to action & state.
- Add simple CLI script action client.

## Risks
- Hardware serial port not found: fallback to simulated.
- Timing drift: discuss why feedback is approximate.
- Action cancel vs emergency stop differences.

## Success Criteria
- Students can send a goal, see feedback, cancel it, and observe state topic.
- Understand mapping from REST background processing -> ROS 2 action.
