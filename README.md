<!-- ===================================================================== -->
# GEM Summoner 🚗 — Autonomous “Summon” Feature for the GEM e2
[![UIUC ECE 484](https://img.shields.io/badge/Course-ECE%20484-orange)](https://ece.illinois.edu/)
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-Academic-lightgrey)](#license)

A full-stack **ROS (Noetic)** system that lets a GEM e2 neighborhood-electric vehicle **drive itself from any parked bay on a closed track to a user-selected GPS waypoint**, while it:

* **Exits parking bays** autonomously  
* **Follows lanes** (straight & curved) with a tuned PID controller  
* **Avoids obstacles** in real time using an **Ouster** LiDAR ROI filter  
* **Stops precisely** at the summon point once position *and* heading criteria are met  

Project by **Neel Patel, Pusan Chakraborty, Rohith Madhavan, Steffen Brown** (UIUC ECE 484, Spring 2025).

---

## 📑 Table of Contents
1. [Project Motivation](#project-motivation)  
2. [System Architecture](#system-architecture)  
3. [Key Algorithms](#key-algorithms)  
4. [Quick Start](#quick-start)  
5. [Repository Layout](#repository-layout)  
6. [Demonstrations](#demonstrations)  
7. [Results & Metrics](#results--metrics)  
8. [Challenges & Lessons Learned](#challenges--lessons-learned)  
9. [Contributors](#contributors)  
10. [License](#license)  

---

## Project Motivation
Valet-style “summon” features usually rely on HD maps and expensive sensors.  
Our mission: **deliver similar user convenience on a low-cost GEM e2** using only

* Phone + on-board GPS,  
* A single **Ouster OS1-128** LiDAR, and  
* Lightweight classical control (no heavyweight ML stack).  

The result is a **portable reference design** suitable for many small-EV platforms.

---

## System Architecture

```mermaid
%% GEM e2 Summon System – GitHub-safe Mermaid (no :::, no mid-block %%)
flowchart LR
    %% ---------- Client side ----------
    subgraph Client_Side["Client&nbsp;Side"]
        A["Web&nbsp;App<br/>React 18"]
        A -- "HTTPS&nbsp;+&nbsp;JWT" --> B["Flask&nbsp;REST&nbsp;API"]
    end

    B -- "rosbridge&nbsp;WebSocket" --> C[rosbridge_server]
    C -- "pub&nbsp;/&nbsp;sub" --> D((roscore))

    %% ---------- Sensors ----------
    subgraph Sensors["On-board&nbsp;Sensors"]
        SC["Stereo&nbsp;Camera"]
        IMU["IMU"]
        LIDAR["Ouster&nbsp;OS1-128"]
        GPS["GPS"]
    end

    %% ---------- Perception / control nodes ----------
    E["Exit-Parking FSM"]
    F["PID&nbsp;Lane-Follow"]
    G["LiDAR ROI<br/>Collision-Stop"]
    H["Arrival&nbsp;Checker"]

    %% ---------- Orchestrator ----------
    SM["SummonManager"]

    %% ---------- Actuation ----------
    VB["Vehicle&nbsp;Base<br/>(PACMod)"]

    %% ---------- Data flows ----------
    D -->|cmd_mux<br/>&amp; state| SM
    SM -->|/enable, /steer,<br/>/accel, /brake| VB

    %% Sensor feeds
    SC --> E & F
    IMU --> E
    LIDAR --> G
    GPS --> SM & H

    %% Module outputs to SummonManager
    E -->|EP_OUTPUT/*| SM
    F -->|LF_OUTPUT/*| SM
    G -->|/OBJECT_DETECTION| SM
    H -->|/ARRIVAL/arrived| SM

    %% Module activations from SummonManager
    SM -->|/EXIT_PARK/active| E
    SM -->|/LANE_DETECTION/active| F

### SummonManager: Orchestration Logic

The `SummonManager` node acts as the central orchestrator, coordinating all autonomous modules based on a finite state machine (FSM). It selectively activates modules, routes PACMod commands, and handles emergency behavior.

---

#### FSM States

| State | Name            | Description                                                                  |
| ----- | --------------- | ---------------------------------------------------------------------------- |
| `0`   | **IDLE**        | Vehicle remains stationary (Park + Brake). Awaiting GPS goal input.          |
| `1`   | **EXIT**        | Exit-Parking FSM is activated. SummonManager relays `/EP_OUTPUT/*` commands. |
| `2`   | **LANE FOLLOW** | PID Lane-Follow is activated. SummonManager relays `/LF_OUTPUT/*` commands.  |

---

### Data Flow Summary

| Source              | Topic                                 | Role                             |
| ------------------- | ------------------------------------- | -------------------------------- |
| Web App             | `/WEBAPP/goal_lat`, `/goal_long`      | Provides target GPS location     |
| GPS Receiver        | `/navsatfix`                          | Supplies current position        |
| Arrival Checker     | `/ARRIVAL/arrived`                    | Triggers final stop condition    |
| Object Detection    | `/OBJECT_DETECTION/stop` / `/restart` | Triggers emergency stop/resume   |
| Exit-Parking Module | `/EP_OUTPUT/*`                        | Sends control commands (State 1) |
| Lane-Follow Module  | `/LF_OUTPUT/*`                        | Sends control commands (State 2) |

---

### Command Multiplexing by FSM State

```plaintext
FSM:        [IDLE]         [EXIT]           [LANE FOLLOW]
------------------------------------------------------------
Enable:     True           EP_ENABLE        LF_ENABLE
Gear:       PARK           EP_GEAR          LF_GEAR
Steer:      0.0            EP_STEER         LF_STEER
Accel:      0.0            EP_ACCEL         LF_ACCEL
Brake:      1.0 (Full)     EP_BRAKE         LF_BRAKE
```

---

### Activation Logic

```python
if fsm_state == 1:
    self.exit_parking_active_pub.publish(True)
    self.lane_detection_active_pub.publish(False)
elif fsm_state == 2:
    self.exit_parking_active_pub.publish(False)
    self.lane_detection_active_pub.publish(True)
else:
    self.exit_parking_active_pub.publish(False)
    self.lane_detection_active_pub.publish(False)
```

> SummonManager switches which module is “active” and routes PACMod commands from that module to `/pacmod/as_rx/*`.

---

### Arrival Detection

Vehicle transitions to IDLE and stops when:

* Distance to target < `4.0 meters`, and
* Heading vector is perpendicular to target vector

These conditions are computed from GPS (`/navsatfix`) and INS heading.

---

### Obstacle Handling

```text
/OBJECT_DETECTION/stop     → Triggers emergency brake
/OBJECT_DETECTION/restart  → Resumes from last FSM state
```

If an obstacle is detected, SummonManager overrides control and holds full brake. Once cleared, FSM resumes automatically.











