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











