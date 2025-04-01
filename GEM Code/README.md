# Summon GEM Project - ECE 484

## File Breakdown

```
my_summon_project/
├── launch/
│   └── summon_system.launch              # Starts all ROS nodes
│
├── scripts/
│   ├── summon_manager_fsm.py            # 🧠 Master FSM for phase control
│   │                                    # Controls mode: EXIT_PARKING, LANE_FOLLOW, INTERSECTION_DECISION, SUMMON_APPROACH, STOP
│   │
│   ├── exit_parking_controller.py       # 🚗 Pulls car straight out of any depth 90° spot
│   ├── intersection_detector.py         # 🔀 Detects when car is at loop entrance/fork
│   ├── intersection_decision.py         # 🧭 Chooses shortest path based on user location
│   │
│   ├── lane_detection.py                # 🛣️ Provided (Stanley/Pure Pursuit lane tracker)
│   ├── summon_approach.py               # 📍 [NEW] Monitors distance to user via GNSS; stops when getting farther
│   │
│   ├── networking_node.py               # 🌐 Connects to Flask API for user GPS
│   ├── obstacle_detector.py             # 🚧 Stops if LiDAR detects close object
│   └── gem_pid.py
│
└── utils/
    ├── filters.py                       # Provided
    └── pid.py               # Provided

```

## Highest Level FSM

![High Level FSM](misc/Highlevel_FSM.png)


## INDIVIDUAL FILE DOCUMENTATION GOES HERE