
# USV Simulation and Control

This repository contains all packages necessary to set up a USV (Unmanned Surface Vehicle) simulation workspace, control the boat using ROS, and access the camera feed for image processing. 

## Getting Started

### 1. Set Up the Workspace

1. **Create the workspace**:
   ```bash
   mkdir -p ~/usv_simulation/src
   cd ~/usv_simulation/src
   ```

2. **Clone this repository** into the `src` folder:
   ```bash
   git clone https://github.com/Vishwajeetiitb/boat_simulation.git
   ```

3. **Build the workspace**:
   ```bash
   cd ~/usv_simulation
   catkin_make
   ```

4. **Source the workspace** to update the ROS environment:
   ```bash
   echo "source ~/usv_simulation/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### 2. Launch the Simulation

To launch the USV simulation in Gazebo with a predefined environment:

```bash
roslaunch vrx_gazebo simulation.launch
```

This command will open Gazebo in the Sydney Regatta environment, where you can simulate and control the boat.

### 3. Running the Boat Control and Camera Feed Script

After launching the simulation, run the control script to operate the boat and access the camera feed:

```bash
rosrun boat_controls interact.py
```

This script will:
- **Control the boat's movement** based on predefined thrust values.
- **Access the front camera feed** for real-time image processing.

In the script, you can add custom image processing code to analyze the camera feed for specific tasks.

### 4. Optional: View Camera Feed

The camera feed from the boat's front camera is available in the simulation and processed within the script. You can view this feed directly in an OpenCV window or modify the `interact.py` script to save or process the images as needed.

## Customizing the Control and Image Processing

1. **Thrust Control**: Modify the thrust values in `interact.py` to change the boat's speed and turning behavior.
2. **Image Processing**: Add image processing functions within the script to detect objects, analyze colors, or perform other computer vision tasks.

### 5. Additional Commands

To reset the simulation or environment state, you can use:
```bash
rosservice call /gazebo/reset_simulation
```

This command is helpful when testing changes in control or when resetting the simulation state.

---

## Requirements

Ensure the following dependencies are installed:
- ROS Noetic
- Gazebo 11
- [cv_bridge](http://wiki.ros.org/cv_bridge)
- [OpenCV](https://opencv.org/)

## Repository Structure

- `boat_controls`: Contains the main control script (`interact.py`) for boat operation and image processing.
- `vrx_gazebo`: Includes simulation packages and launch files for different environments and scenarios.

---

### Troubleshooting

- **Environment Setup**: If the `catkin_make` command fails, verify your ROS and workspace setup. Ensure all necessary dependencies are installed.
- **Gazebo Performance**: For smoother simulations, close other heavy applications and reduce the simulation resolution in Gazebo if needed.

---

Happy Simulating! 🎉