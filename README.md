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
flowchart LR
    %% ──────────────────────────────────────────────
    %%  Client side
    %% ──────────────────────────────────────────────
    subgraph Client["Client&nbsp;Side"]
        direction LR
        WebApp["Web&nbsp;App<br/>React&nbsp;18"]
        WebApp -- "HTTPS&nbsp;+&nbsp;JWT" --> API["Flask&nbsp;REST&nbsp;API"]
    end
    API -- "rosbridge&nbsp;WebSocket" --> Bridge[rosbridge_server]
    Bridge -- "pub&nbsp;/&nbsp;sub" --> Core((roscore))

    %% ──────────────────────────────────────────────
    %%  Sensors (ordered to align with modules)
    %% ──────────────────────────────────────────────
    subgraph Sensors["On-board&nbsp;Sensors"]
        direction TB
        SC["Stereo&nbsp;Camera"]
        IMU["IMU"]
        LIDAR["Ouster&nbsp;OS1-128"]
        GPS["GPS"]
    end

    %% ──────────────────────────────────────────────
    %%  Modules (ordered to minimise crossings)
    %% ──────────────────────────────────────────────
    subgraph Modules["Perception&nbsp;/&nbsp;Control&nbsp;Nodes"]
        direction TB
        LF["PID&nbsp;Lane-Follow"]      %% needs Stereo
        EP["Exit-Parking&nbsp;FSM"]     %% needs Stereo + IMU
        ROI["LiDAR&nbsp;ROI<br/>Collision-Stop"]  %% needs LiDAR
        ARR["Arrival&nbsp;Checker"]     %% needs GPS
    end

    %% ──────────────────────────────────────────────
    %%  Orchestrator & Actuation
    %% ──────────────────────────────────────────────
    SM["SummonManager"]:::orchestrator
    VB["Vehicle&nbsp;Base<br/>(PACMod)"]:::vehicle

    %% Sensor → module feeds (parallel arrows, minimal crossing)
    SC  --> LF
    SC  --> EP
    IMU --> EP
    LIDAR --> ROI
    GPS --> ARR
    GPS --> SM

    %% Module → SummonManager (status / cmd outputs)
    LF  -->|LF_OUTPUT|  SM
    EP  -->|EP_OUTPUT|  SM
    ROI -->|/OBJECT_DETECTION| SM
    ARR -->|/ARRIVAL/arrived|  SM

    %% SummonManager → module activations
    SM -->|/LANE_DETECTION/active| LF
    SM -->|/EXIT_PARK/active|     EP

    %% SummonManager → Vehicle Base
    SM -->|/pacmod/as_rx/*| VB

    %% Visual groups
    classDef orchestrator fill:#d9d9ff,stroke:#333;
    classDef vehicle      fill:#ffd9d9,stroke:#333;








