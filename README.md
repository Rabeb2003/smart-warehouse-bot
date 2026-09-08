# Smart Warehouse Bot

![Warehouse Simulation](warehouse_bot.png)

![Warehouse Bot Simulation](warehouse-bot_simulation.png)

![Gazebo + RViz2](gazebo+rviz2.png)

Autonomous mobile robot system for warehouse inspection missions using ROS2 Humble, Nav2 navigation stack, and computer vision-based station verification with ArUco markers.

## Research Context

Autonomous warehouse robots require robust navigation in structured environments with precise station verification for inventory management and logistics operations. This project addresses the challenge of autonomous waypoint navigation with visual confirmation at inspection stations, integrating SLAM-based localization, trajectory planning, and fiducial marker detection in a simulated warehouse environment.

## Technical Contributions

- **Mission Orchestration**: Sequential state machine for 5-step autonomous inspection mission with Nav2 action client integration
- **Visual Station Verification**: Real-time ArUco marker detection (DICT_5X5_50) with OpenCV for station confirmation and timestamp-based verification
- **Integrated Navigation Stack**: Complete Nav2 configuration with AMCL localization, NavFn global planner, and DWB local planner tuned for narrow warehouse aisles
- **Simulation Environment**: Gazebo-based warehouse simulation with AWS RoboMaker models and custom ArUco marker placements
- **Vision-Based Docking**: Preliminary implementation of visual docking controller for precise charging station alignment

## Technical Stack

| Component | Technology | Version |
|-----------|------------|---------|
| Framework | ROS 2 | Humble Hawksbill |
| Navigation | Nav2 | Humble |
| Localization | AMCL | Built-in |
| SLAM | Cartographer | ROS2 |
| Simulation | Gazebo Classic | 11 |
| Robot Platform | TurtleBot3 | Burger with Camera |
| Computer Vision | OpenCV | 4.x |
| Fiducial Markers | ArUco | DICT_5X5_50 |
| Planning | NavFn / DWB | Nav2 |
| Language | Python | 3.10 |

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Mission Orchestrator                       │
│              (warehouse_autonomy_node.py)                    │
└────────────────────┬────────────────────────────────────────┘
                     │
        ┌────────────┴────────────┐
        │                         │
        ▼                         ▼
┌───────────────┐         ┌───────────────┐
│   Nav2 Stack  │         │  Perception   │
│  (NavigateTo  │◄────────┤  (ArUco Detect)│
│    Pose)      │         │               │
└───────┬───────┘         └───────────────┘
        │
        ├──────────────────┐
        │                  │
        ▼                  ▼
┌───────────────┐  ┌───────────────┐
│  Global Planner│  │ Local Planner │
│    (NavFn)     │  │    (DWB)      │
└───────┬───────┘  └───────┬───────┘
        │                  │
        └────────┬─────────┘
                 │
                 ▼
        ┌───────────────┐
        │  Costmaps     │
        │ (Global/Local)│
        └───────┬───────┘
                │
        ┌───────┴────────┐
        │                │
        ▼                ▼
┌───────────────┐  ┌───────────────┐
│     AMCL      │  │   Gazebo      │
│ (Localization)│  │  Simulation   │
└───────┬───────┘  └───────┬───────┘
        │                  │
        └────────┬─────────┘
                 │
                 ▼
        ┌───────────────┐
        │   Sensors     │
        │ (LiDAR/Camera)│
        └───────────────┘
```

## Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **Dependencies**:
  ```bash
  sudo apt install ros-humble-desktop
  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
  sudo apt install ros-humble-turtlebot3*
  sudo apt install ros-humble-turtlebot3-simulations
  sudo apt install ros-humble-gazebo-ros-pkgs
  sudo apt install python3-opencv ros-humble-cv-bridge
  sudo apt install ros-humble-dynamixel-sdk
  sudo apt install ros-humble-turtlebot3-msgs
  pip3 install colcon-common-extensions
  ```

## Build & Run

### Clone Repository

```bash
git clone https://github.com/Rabeb2003/smart-warehouse-bot.git
cd smart-warehouse-bot
```

### Build Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### Launch Simulation

```bash
# Terminal 1: Launch complete system (Gazebo + Nav2 + Perception)
export TURTLEBOT3_MODEL=burger_cam
ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py
```

### Run Autonomous Mission

```bash
# Terminal 2: Launch mission orchestrator (after 60 seconds)
source install/setup.bash
ros2 run warehouse_bringup warehouse_autonomy
```

### Expected Demo

The robot will:
1. Initialize in Gazebo warehouse environment
2. Load pre-built SLAM map and start AMCL localization
3. Execute 5-step autonomous inspection mission:
   - Navigate to Home Charging Dock (verify ArUco ID 0)
   - Navigate to Loading Zone (verify ArUco ID 1)
   - Navigate to Inventory Scan Station (verify ArUco ID 2)
   - Navigate to Dispatch Zone (verify ArUco ID 3)
   - Return to Home Charging Dock (verify ArUco ID 0)
4. Display mission progress and verification status in terminal

## Validation & Results

### Qualitative Validation

- **Navigation Success**: Robot successfully navigates between all 5 waypoints in simulation
- **ArUco Detection**: Markers detected reliably within 2 seconds at each station
- **Mission Completion**: Full 5-step mission executes autonomously without human intervention
- **Localization**: AMCL maintains stable localization throughout mission

### Performance Metrics (Simulation)

| Metric | Value |
|--------|-------|
| Mission Duration | ~3-5 minutes |
| Waypoint Navigation Time | 30-60 seconds per waypoint |
| ArUco Detection Time | < 2 seconds |
| Positioning Accuracy | ±5 cm (xy_goal_tolerance) |
| Detection Rate | 100% (4/4 stations) |

### Limitations

- **Simulation Only**: System validated only in Gazebo simulation; real-world deployment requires sensor calibration and environmental adaptation
- **Static Environment**: Assumes static warehouse layout; dynamic obstacles not handled
- **Single Robot**: No multi-robot coordination implemented
- **Marker Dependency**: Mission verification relies on ArUco markers; alternative verification methods not explored

## Research Relevance

This project demonstrates competence in:

1. **ROS2 Navigation Stack**: Complete integration of Nav2 components with custom tuning for structured environments
2. **Perception-Planning Integration**: Computer vision verification tightly coupled with navigation actions
3. **Simulation-to-Real Pipeline**: Gazebo-based development workflow transferable to physical robots
4. **System Integration**: End-to-end autonomous system from sensors to mission orchestration

## Author

**Rabeb Bouzaida**  
Electrical Engineering, ENIM (Tunisia)  
GitHub: [Rabeb2003](https://github.com/Rabeb2003)

## License

MIT License - see [LICENSE](LICENSE) file for details

## Citation

If you use this project for research, please cite:

```bibtex
@software{smart_warehouse_bot,
  author = {Bouzaida, Rabeb},
  title = {Smart Warehouse Bot},
  year = {2024},
  url = {https://github.com/Rabeb2003/smart-warehouse-bot}
}
```

## Acknowledgments

- ROS 2 Humble and Nav2 communities
- TurtleBot3 project
- AWS RoboMaker warehouse world models
- OpenCV and ArUco documentation
