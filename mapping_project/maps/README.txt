Sample Maps for ROS Mapping Project

This directory contains sample map files that demonstrate what you would get after running the mapping process with either a physical robot or in simulation.

To generate your own maps:

1. Using Physical Robot:
   - Launch the mapping process: roslaunch mapping_project mapping.launch
   - Control the robot to explore the environment
   - Save the map: rosrun map_server map_saver -f ~/map

2. Using Simulation:
   - Launch the simulation: roslaunch mapping_project simulation_mapping.launch
   - Control the robot using keyboard (w,a,s,d)
   - Save the map: rosrun map_server map_saver -f ~/map

The saved map will consist of two files:
- map.pgm: The actual map image
- map.yaml: Configuration file containing map metadata

Map File Format:
- Black pixels (0): Occupied space (walls, obstacles)
- White pixels (255): Free space
- Gray pixels (205): Unknown space

To use these maps for navigation or other purposes, place them in your ~/.ros directory or specify the full path when loading them. 