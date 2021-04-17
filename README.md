# simple-reachability
![](https://img.shields.io/badge/ROS-noetic-success)

This package allows you to visualize the workspace of your robot easily. The calculation will be done using the **moveit** framwork. So the only thing you need is a **moveit_config** of your robot.

### Preparation

Before launching the **simple-reachability** nodes launch the *demo.launch* file from your *moveit* configuration to provide a planning interface and open up **RVIZ**.

```roslaunch my_robot_moveit_config demo.launch```

**Important:** Configure the *config* files in the *config* folder of the **simple-reachability** folder to get the result you like. There is one for each node. You will find a documentation of all parameters in the configuration file.

### Usage

##### Calculation

To run the calculation of the workspace for your robot, run:

```roslaunch simple-reachability calculate_workspace```

This will calculate the workspace in the way you defined in the *calculation.yaml*. You are for example free to calculate the workspace *reachable* workspace* or the workspace with a given end-effector orientation.

The calculation could take some time depending on the resolution. The node will let you see the current progress and the estimated time to finish.

After the calculation has finished, the result will be **published** on the ```/calculate_workspace/visualization_marker``` topic. You can easily visualize it by adding the display type *Marker* in RVIZ.

To not loose the workspace calculation result, the node wil **save** it in a **rosbag** in the **simple-reachability** bags folder with an unique name.

##### Visualization

If you want to visualize a previously calculated workspace, you can run:

```roslaunch simple-reachability visualize_workspace.launch```

Make sure you provide the name of the file you want to run in the *visualization.yaml*.

Now you can visualize the workspace by adding a *Marker* with the topic ```/visualize_workspace/visualization_marker``` in RVIZ.
