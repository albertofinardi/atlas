# ATLAS: Autonomous Traffic Light And Sign System

A comprehensive ROS2-based solution for autonomous robot navigation with traffic sign and signal recognition capabilities. ATLAS implements robust line-following using computer vision and PID control, along with traffic element recognition using ArUco markers.

## System Architecture

The system consists of three primary ROS2 nodes:

- **Line PID Node** (`line_pid_node.py`): Processes camera images to detect lines and calculate steering commands
- **Traffic Detection Node** (`traffic_detection_node.py`): Analyzes camera input to recognize traffic signs and signals using ArUco markers
- **Controller Node** (`controller_node.py`): Integrates information from other nodes to control robot movement

## Project Structure

```
atlas/
├── models/              # Robot models and additional vision sensors (root level)
├── scenes/              # Complete test scene with circuit and traffic signs (root level)
├── media/               # Videos showcasing the robot
├── utils/               # ArUco generation code and assets
├── src/
│   └── atlas/           # ROS2 package directory
│       ├── launch/      # ROS2 launch files
│       │   ├── controllers.launch        # Main launch file
│       │   └── controllers_debug.launch  # Debug launch with visualization
│       ├── atlas/       # Project sourcecode
│       │   ├── line_pid_node.py
│       │   ├── traffic_detection_node.py
│       │   └── controller_node.py
│       ├── setup.py     # Python package setup
│       └── package.xml  # ROS2 package configuration
├── pixi.toml           # Pixi configuration file
└── README.md
```

## Getting Started

### Installation

This project uses Pixi for dependency management.

1. **Install dependencies**:
```bash
pixi install
```

**Note**: If you get a file thread error, allow the system to open more files simultaneously in this terminal using:
```bash
ulimit -n 4096
```

2. **Build the ROS2 workspace**:
```bash
colcon build --symlink-install
```

3. **Source the workspace**:
```bash
source install/setup.bash
```

4. **Load simulation models and scenes**:
   - Models are located in the root `models/` directory
   - Test scenes are located in the root `scenes/` directory
   - Copy models to your CoppeliaSim models folder or reference them directly
   - Load the test scene from `scenes/` directory in CoppeliaSim

## Running the System

**Important**: Do not forget to run `source install/setup.bash` in each new one pixi shell.

### Basic Launch

Launch the complete ATLAS system:
```bash
ros2 launch atlas controllers.launch name:=/rm0
```

### Debug Mode

Launch with visualization and debug topics:
```bash
ros2 launch atlas controllers_debug.launch name:=/rm0
```

### Individual Nodes

Run nodes individually for testing:

```bash
# Line detection node
ros2 run atlas line_pid_node

# Traffic detection node  
ros2 run atlas traffic_detection_node

# Controller node
ros2 run atlas controller_node
```

## Configuration

### Parameters

Check the report for a full explanation of every parameter and their impact on system behavior.

## ROS2 Topics

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `cmd_vel` | `geometry_msgs/Twist` | Velocity commands for robot |
| `line_pid/steering` | `std_msgs/Float32` | Normalized steering values [-1.0, +1.0] |
| `line_pid/stop_line_detected` | `std_msgs/Bool` | Stop line detection status |
| `line_pid/lost_line_detected` | `std_msgs/Bool` | Lost line detection status |
| `traffic/id` | `std_msgs/String` | Detected traffic sign identifier |

### Debug Topics (when enabled)

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `line_pid/debug/left_cam` | `sensor_msgs/Image` | Left camera debug feed |
| `line_pid/debug/center_cam` | `sensor_msgs/Image` | Center camera debug feed |
| `line_pid/debug/right_cam` | `sensor_msgs/Image` | Right camera debug feed |
| `traffic/debug/overlay` | `sensor_msgs/Image` | Traffic detection overlay |
| `traffic/debug/processed` | `sensor_msgs/Image` | Processed binary image |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/vision/left` | `sensor_msgs/Image` | Left camera feed |
| `/vision/center` | `sensor_msgs/Image` | Center camera feed |
| `/vision/right` | `sensor_msgs/Image` | Right camera feed |
| `camera/image_color` | `sensor_msgs/Image` | Main camera for traffic detection |

## Traffic Sign Configuration

The system recognizes the following ArUco markers:

| Marker ID | Traffic Element |
|-----------|----------------|
| 0 | Red traffic light |
| 1 | Yellow traffic light |
| 2 | Green traffic light |
| 3 | Speed limit 30 km/h |
| 4 | Speed limit 50 km/h |

The code to generate ArUco markers is located in the `utils/` folder, together with the ArUco assets.

## Test Scene Setup

**A full scene that can be run to evaluate the robot is present in `scenes/`, called `Circuit.ttt`.**

### Custom CoppeliaSim Configuration

1. **Load the robot model**:
   - Models are located in the root `models/` directory of the project
   - Use the provided RoboMaster S1 model with additional vision sensors
   - Ensure the model includes left, center, and right vision cameras

2. **Create a custom scene**:
   - Use the provided test scenes as reference (it includes already the model)
   - Design your own circuit with traffic signs and navigation challenges

3. **Traffic light cube orientation**:
   - **Important**: Traffic light cubes must be oriented correctly
   - Incorrect rotation causes face misalignment and detection failures
   - Verify cube faces are properly aligned with marker patterns

4. **Speed limits**:
   - No default speed limit sign is provided as model, but can easily constructed applying a texture on a `Plane`, using assets from `utils/aruco_markers/`.

### Vision System Setup

The vision system is implemented in CoppeliaSim using Lua scripts:
- Creates absolute topic paths: `/vision/left`, `/vision/center`, `/vision/right`
- For multi-robot simulations, modify Lua scripts and Line PID Node for unique topic names

## Monitoring and Debugging

### Visualization Tools

When running in debug mode, you can monitor the system using RQT.

## Important Notes

### Multi-Robot Deployment

- Vision sensor topics use absolute paths (`/vision/*`)
- For multiple robots, modify:
  - Lua scripts to generate unique topic names per robot
  - Line PID Node to subscribe to correct robot-specific topics
  - Launch files to use relative paths with robot namespaces

### Performance Considerations

- **Speed vs. Accuracy**: Lower speeds provide better line following accuracy
- **Lighting Sensitivity**: System performs well under moderate lighting variations
- **Processing Latency**: Real-world deployment may face increased latency

## Troubleshooting

### Common Issues

1. **No line detection**:
   - Check camera feeds are publishing
   - Verify threshold values for current lighting
   - Ensure line contrast is sufficient

2. **Traffic sign not detected**:
   - Verify ArUco marker orientation
   - Check detection confidence parameters
   - Ensure adequate marker size and distance

3. **Robot not moving**:
   - Check startup delay has elapsed and the drivers are set up correctly. If no message is print from the drivers, redo the installation process and make sure to have sourced the install bash file.
   - Verify cmd_vel topic is being published
   - Ensure no stop conditions are active