# Autonomous-Safe-Landing-Drone

Perform master and slave operation to the companion-computer(nuc) and your computer.
save this in a workspace(safe_landing_ws/src) and do catkin build\

## How To Run software
### Terminal 1:-
```
roslaunch px4 mavros_posix_sitl.launch
```
### Terminal 2:-
```
source devel/setup.bash
roslaunch safe_landing_planner safe_landing_planner.launch
```
### Terminal 3:-
```
source devel/setup.bash
roslaunch depth_image_proc point_cloud_xyz.launch
```
### Terminal 4:-
```
source devel/setup.bash
roslaunch offboard_control controller_pid.launch
```
### Terminal 5:-
```
rostopic pub /target_pos geometry_msgs/PoseStamped "header:r:
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:me_id: ''
  position:
    x: 0.0:
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" --once
publish target want to fly then trigger safe landing.
rostopic pub /trigger_safe_landing std_msgs/Bool "data: true" --once
```
### Note:-
```
set  COM_OBS_AVOID = 1 in QGC parameters\
change pose of camera in iris_model file to <pose>0 0 -0.04 0 1.5707 0</pose>
```

## How To Run Hardware

### Terminal 1:-
```
roslaunch mavros px4.launch
```
### Terminal 2:-
```bash
cd catkin_ws/
source devel/setup.bash
roslaunch safe_landing_planner safe_landing_planner.launch
```

Install intel realsense package for depth camera and run demo_pointcloud launch file 

### Terminal 3:-
```bash
cd intel_ws/
source devel/setup.bash
roslaunch realsense2_camera rs_camera.launch
```
check rostopic list whether all the topics of camera are updating\
Open rviz and rqt in your computer or do ssh-X for one terminal

### Terminal 4:-
```bash
source devel/setup.bash
roslaunch offboard_control controller_pid.launch
```

check if desire file has all the libraries\
desire can also be put in a separate catkin package\

### Terminal 5:-
```
rostopic pub /target_pos geometry_msgs/PoseStamped "header:r:
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:me_id: ''
  position:
    x: 0.0:
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0ion:
    y: 0.0
    z: 0.0
    w: 0.0" --once
publish target want to fly then trigger safe landing.
rostopic pub /trigger_safe_landing std_msgs/Bool "data: true" --once
```

### Note:-
safe_landing_planner_launch.launch launches the required nodes(safe_landing_planner and waypoint_generator)\
main instructions is to source respective mavros folder- map is changed to local_origin \
set  COM_OBS_AVOID = 1 in QGC parameters\
check rviz folder without flying -  for green zone\
change params in rqt to set the tuning for safe landing\

![](https://github.com/Garuda-IIITH-RRC/Autonomous_Safe_Landing_Drone_Hardware/blob/master/rqt.png)

### Youtube link:-

[![](https://img.youtube.com/vi/pO-g0E1Fz34/0.jpg)](https://www.youtube.com/watch?v=pO-g0E1Fz34)
