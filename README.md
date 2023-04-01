# MonashRA
Official Repository for the Trajectory Planning for Robotic Manipulators Project - Research Assistant (Monash University)


To run Gazebo, MoveIt! and RViz, enter trajopt_ws and run:

Terminal 1: Gazebo
```
roslaunch loading_test shelf_sim.launch
```

Terminal 2: Fetch MoveIt!
```
roslaunch fetch_moveit_config move_group.launch
```

Terminal 3: Rviz, then import .scene in Scene Objects
```
rosrun rviz rviz -d "fetch.rviz"
```

Terminal 4: Your Script!
```
python script.py
```

