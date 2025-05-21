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
flowchart TD
    %% ─────────────────────────────────────────────────────
    %%  Client side  (row 1)
    %% ─────────────────────────────────────────────────────
    subgraph Client["Client&nbsp;Side"]
        direction LR
        WebApp["Web&nbsp;App<br/>React&nbsp;18"]
        WebApp -- "HTTPS&nbsp;+&nbsp;JWT" --> API["Flask&nbsp;REST&nbsp;API"]
    end
    API -- "rosbridge&nbsp;WS" --> Bridge[rosbridge_server]
    Bridge -- "pub&nbsp;/&nbsp;sub" --> Core((roscore))

    %% ─────────────────────────────────────────────────────
    %%  On-board sensors  (row 2)
    %% ─────────────────────────────────────────────────────
    subgraph Sensors["On-board&nbsp;Sensors"]
        direction LR
        SC["Stereo&nbsp;Camera"]
        IMU["IMU"]
        LIDAR["Ouster&nbsp;OS1-128"]
        GPS["GPS"]
    end

    %% ─────────────────────────────────────────────────────
    %%  Perception & control modules  (row 3)
    %% ─────────────────────────────────────────────────────
    subgraph Modules["Perception&nbsp;&nbsp;/&nbsp;&nbsp;Control"]
        direction LR
        EP["Exit-Parking<br/>FSM"]
        LF["PID&nbsp;Lane-Follow"]
        ROI["LiDAR&nbsp;ROI<br/>Collision-Stop"]
        ARR["Arrival&nbsp;Checker"]
    end

    %% ─────────────────────────────────────────────────────
    %%  Orchestrator + actuation  (row 4)
    %% ─────────────────────────────────────────────────────
    SM["SummonManager"]:::orchestrator
    VB["Vehicle&nbsp;Base<br/>(PACMod)"]:::vehicle

    %% ───── Sensor → module feeds ─────
    SC  --> EP & LF
    IMU --> EP
    LIDAR --> ROI
    GPS --> ARR & SM

    %% ───── Module → SummonManager (status / commands) ─────
    EP  -->|EP_OUTPUT| SM
    LF  -->|LF_OUTPUT| SM
    ROI -->|/OBJECT_DETECTION| SM
    ARR -->|/ARRIVAL/arrived| SM

    %% ───── SummonManager → module activations ─────
    SM -->|/EXIT_PARK/active| EP
    SM -->|/LANE_DETECTION/active| LF

    %% ───── SummonManager → Vehicle Base (PACMod) ─────
    SM -->|/pacmod/as_rx/*| VB

    %% ─────────────────────────────────────────────────────
    %%  Styling helper classes
    %% ─────────────────────────────────────────────────────
    classDef orchestrator fill:#d9d9ff,stroke:#333;
    classDef vehicle      fill:#ffd9d9,stroke:#333;






