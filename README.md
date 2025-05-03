# Variable Grasp Pose and Commitment for Trajectory Optimization

This is a research project conducted by [Jiahe Pan](https://mpan31415.github.io/) at Monash University, Australia, under supervision of Prof. Akansel Cosgun. We propose enhancing trajectory optimization for robot manipulation with variable grasp pose sampling using Bayesian optimization. An SQP-based trajectory optimizer from [TrajOpt](https://journals.sagepub.com/doi/full/10.1177/0278364914528132?casa_token=wK3SAC3AeT8AAAAA%3AZKqFj1RBYLYo84QEE0XMQooUYxiato4guT5BvIAPgF28PW1N5ahhPmVHPUJhDuZxe5gPgABcjcV2fg) was integrated into the overall optimization pipeline. Extensive Evaluations were conducted both in simulation (Gazebo) and the real-world on the Fetch mobile robot.

## Paper and Citation Info

The paper was accepted as a full paper at IEEE HORA 2023, and can be found [here](https://ieeexplore.ieee.org/abstract/document/10155773).
If you find our work useful, please consider citing it:
```
@inproceedings{pan2023variable,
  title={Variable Grasp Pose and Commitment for Trajectory Optimization},
  author={Pan, Jiahe and He, Kerry and Ong, Jia Ming and Cosgun, Akansel},
  booktitle={2023 5th International Congress on Human-Computer Interaction, Optimization and Robotic Applications (HORA)},
  pages={1--6},
  year={2023},
  organization={IEEE}
}
```

## Workspaces

The main package directories of this repository are located in the `/moveit_ws/src` directory, and the important ones are summarized below:

### 1. `fetch_ros`
This directory contains the ROS packges of the Fetch robot, such as mapping, navigation, and control. Specifically, the `fetch_moveit_config` package enables the use of MoveIt motion planners with the Fetch robot, and contains the launch files used to start the MoveIt planning server.

### 2. `loading_test`
This package contains the main scripts of the key algorithmic implementations, and the launch files required to start the various modules of the optimization algorithm. 

### 3. `models`
This directory contains the 3D geometry files of various objects, which can be directly loaded into the MoveIt planning scene. Note that various file formats are supported, such as `stl` files and Solidworks parts. This is useful for creating custom environments of the planning workspace.


## Running Experiments

A laptop with <strong>Ubuntu 18.04</strong> and <strong>ROS (Melodic)</strong> installations is required.

The steps followed to run experiments in both simulation and on real hardware are **detailed in `experiments.txt`**.

Below is an example workflow in Gazebo simulation:
1. Launch Gazebo scene:
```
roslaunch loading_test gazebo_sim.launch
```

2. Launch the move_group server (which launches RViz too):
```
roslaunch fetch_moveit_config move_group.launch
```

3. Launch the TrajOpt server node:
```
roslaunch tesseract_ros_examples fetch_simulation.launch
```

4. Launch the BayesOpt client node:
```
rosrun tesseract_ros_examples bayesian_client.py
```

5. Run the master script:
```
python [script].py     # script to run perception and optimization
```

