# AMR(Autonomous Mobile Robot) System

This branch for multi-robot.

If you want to use this repository for a single robot, check out the `master` branch.

## System
<p align="center">
  <img src="https://user-images.githubusercontent.com/82020865/186935476-fbb8fae4-c243-412c-a0be-0c5dd2163d71.png" width="640px"/>
</p>

## Nodes
| Node | Method |
| ------ | ------|
| localizer | emcl: mcl with expansion resetting |
| global_path_planner | A* search algorithm |
| local_map_creator | Ray casting update |
| local_path_planner | DWA: Dynamic Window Approach |

## Installation
```
cd <YOUR_CATKIN_WS>/src
git clone -b multi_robots --depth=1 https://github.com/ToshikiNakamura0412/AMR_System.git
catkin build amr_system
```

## Run
```
roslaunch amr_system all.launch
```
