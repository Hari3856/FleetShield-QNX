# FleetShield
### Distributed Collision Prevention and Fail-Safe Coordination for Autonomous Robot Fleets

**Platform:** QNX Neutrino RTOS  
**Hardware:** Raspberry Pi 5, Autonomous Rover Platforms  
**Development Environment:** QNX Momentics IDE  
**Programming Language:** C  
**Communication:** QNX Native IPC

---

## 1. Overview

FleetShield is a QNX-based distributed coordination and safety system designed for multiple autonomous rovers operating within a shared workspace.

The system focuses on preventing collisions between autonomous robots, monitoring process health, detecting communication failures and maintaining safe operating conditions.

A centralized Fleet Manager coordinates the participating rovers by receiving their status reports, validating incoming messages and issuing movement commands. Independent watchdog mechanisms monitor process and sensor health to support fault detection and fail-safe operation.

FleetShield uses QNX Neutrino RTOS capabilities to demonstrate inter-process communication, real-time scheduling, heartbeat monitoring and process management in a distributed robotic environment.

## 2. Problem Statement

When multiple autonomous robots operate in a shared workspace, simultaneous movement can result in collisions, unsafe proximity and coordination failures.

Communication loss, stale telemetry, sensor faults or unresponsive processes can further increase operational risks.

FleetShield addresses these challenges through:

- Centralized fleet coordination.
- Real-time rover status communication.
- Collision-risk detection and prevention.
- Heartbeat-based process monitoring.
- Safety-state management.
- Watchdog-based fault detection.
- Process recovery integration.

## 3. System Architecture

FleetShield consists of independently executing QNX processes.

### Core Components

**1. Fleet Manager**
- Acts as the central coordination process.
- Receives rover telemetry through QNX IPC.
- Validates incoming messages.
- Checks message freshness using sequence numbers and timestamps.
- Maintains fleet safety states.
- Issues movement commands such as MOVE, SLOW, STOP and RESUME.
- Monitors rover heartbeats.

**2. Rover Application**
- Represents an individual autonomous rover process.
- Communicates with the Fleet Manager.
- Sends position, movement, sensor and heartbeat information.
- Receives movement and safety commands.

**3. Fleet Watchdog**
- Monitors the Coordination Manager and both rover processes.
- Uses heartbeat messages to track process health.
- Validates process IDs, sequence numbers and timestamps.
- Detects missed heartbeats.
- Reports heartbeat restoration.

**4. Rover Watchdog**
- Provides the sensor-monitoring interface for rover safety.
- Defines sensor reports, sensor validity, fault classifications and rover position information.

### Architecture Diagram

```text
                 FLEETSHIELD SYSTEM
                         |
          +--------------+--------------+
          |                             |
    FLEET MANAGER                 FLEET WATCHDOG
          |                             |
    QNX IPC Server              Process Monitoring
          |                     Heartbeat Detection
     +----+----+                Fault Reporting
     |         |
  ROVER 1   ROVER 2
     |         |
  Telemetry  Tele
