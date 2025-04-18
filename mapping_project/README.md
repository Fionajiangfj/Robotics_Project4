# ROS Mapping Project

This project implements a 2D mapping system using ROS and a robot equipped with LiDAR. The system uses the gmapping package to create a map of the environment. It can be used with either a physical robot or in simulation using Gazebo.

## Prerequisites

- ROS (tested with ROS Noetic)
- For physical robot:
  - Turtlebot3 (or compatible robot with LiDAR)
  - gmapping package
  - rviz
- For simulation:
  - Gazebo
  - turtlebot3_gazebo package
  - turtlebot3_description package
  - turtlebot3_navigation package

## Installation

1. Install required packages:
```bash
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-description ros-noetic-turtlebot3-navigation
```

2. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone <repository-url>
```

3. Build the package:
```bash
cd ~/catkin_ws
catkin_make
```

4. Source the workspace:
```bash
source devel/setup.bash
```

## Usage

### Physical Robot
1. Start the mapping process:
```bash
roslaunch mapping_project mapping.launch
```

2. Control the robot manually to explore the environment. The map will be built in real-time and displayed in RViz.

### Simulation
1. Start the simulation and mapping process:
```bash
roslaunch mapping_project simulation_mapping.launch
```

2. Control the robot using the keyboard:
   - w: move forward
   - s: move backward
   - a: turn left
   - d: turn right
   - x: stop
   - space: force stop

3. The map will be built in real-time and displayed in RViz.

### Saving the Map
To save the map, open a new terminal and run:
```bash
rosrun map_server map_saver -f ~/map
```

This will save two files:
- `~/map.pgm`: The map image
- `~/map.yaml`: The map configuration file

## Configuration

The mapping parameters can be adjusted in the launch files. Key parameters include:
- `map_update_interval`: How often to update the map
- `maxUrange`: Maximum usable range of the laser
- `delta`: Resolution of the map
- `particles`: Number of particles in the particle filter

## Visualization

The map can be visualized in real-time using RViz. The configuration file `mapping.rviz` is included and will be loaded automatically when launching the mapping process.

## Troubleshooting

1. If the map is not updating:
   - Check if the LiDAR is publishing data on the `/scan` topic
   - Verify that the robot's odometry is working correctly
   - Ensure the TF tree is properly configured

2. If the map quality is poor:
   - Adjust the gmapping parameters in the launch file
   - Move the robot more slowly
   - Ensure the environment has enough features for the LiDAR to detect

3. For simulation issues:
   - Make sure all required packages are installed
   - Check if Gazebo is properly installed and configured
   - Verify that the Turtlebot3 packages are correctly installed 