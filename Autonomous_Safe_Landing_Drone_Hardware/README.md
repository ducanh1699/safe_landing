# Autonomous-Safe-Landing-Drone

Perform master and slave operation to the companion-computer(nuc) and your computer.
save this in a workspace(safe_landing_ws/src) and do catkin build\

## How To Run Hardware

### Terminal 1:-
```bash
cd safe_landing_ws/
source devel/setup.bash
roslaunch mavros px4.launch
```
### Terminal 2:-
```bash
cd catkin_ws/
source devel/setup.bash
roslaunch safe_landing_planner safe_landing_planner_launch.launch
```

Install intel realsense package for depth camera and run demo_pointcloud launch file 

### Terminal 3:-
```bash
cd intel_ws/
source devel/setup.bash
roslaunch realsense2_camera demo_pointcloud.launch
```
check rostopic list whether all the topics of camera are updating\
Open rviz and rqt in your computer or do ssh-X for one terminal

### Terminal 4:-
```bash
cd safe_landing_ws/
source devel/setup.bash
rosrun safe_landing_planner desire.cpp
```
check if desire file has all the libraries\
desire can also be put in a separate catkin package\

### Note:-
safe_landing_planner_launch.launch launches the required nodes(safe_landing_planner and waypoint_generator)\
main instructions is to source respective mavros folder- map is changed to local_origin \
set  COM_OBS_AVOID = 1 in QGC parameters\
check rviz folder without flying -  for green zone\
change params in rqt to set the tuning for safe landing\

![](https://github.com/Garuda-IIITH-RRC/Autonomous_Safe_Landing_Drone_Hardware/blob/master/rqt.png)

### Youtube link:-

[![](https://img.youtube.com/vi/pO-g0E1Fz34/0.jpg)](https://www.youtube.com/watch?v=pO-g0E1Fz34)
