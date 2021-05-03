# simple-reachability
![](https://img.shields.io/badge/ROS-noetic-success)

This package allows you to visualize the workspace of your robot easily considering collision detection and joint limits. The calculation will be done using the **moveit** framwork. So the only thing you need is a **moveit_config** of your robot.

### Preparation

Before launching the **simple-reachability** nodes launch the *demo.launch* file from your *moveit* configuration to provide a planning interface and open up **RVIZ**.

```roslaunch my_robot_moveit_config demo.launch```

**Important:** Configure the *config* files in the *config* folder of the **simple-reachability** folder to get the result you like. There is one for each node. You will find a documentation of all parameters in the configuration file.

### Usage

##### Calculation

To run the calculation of the workspace for your robot, run:

```roslaunch simple-reachability calculate_workspace```

This will calculate the workspace in the way you defined in the *calculation.yaml*. You only have o provide an end-effector orientation and then can calculate the workspace with a given end-effector orientation.

The calculation could take some time depending on the resolution. The node will let you see the current progress and the estimated time to finish. If you want to speed up the calculation you could use *IKFast* to make *moveit* find a valid plan a lot faster.

After the calculation has finished, the result will be **published** on the ```/calculate_workspace/visualization_marker``` topic. You can easily visualize it by adding the display type *Marker* in RVIZ.

To not loose the workspace calculation result, the node wil **save** it in a **rosbag** in the **simple-reachability** bags folder with an unique name.

Hint: After changing the config files, you should restart your roscore to reset the ROS params. Think about embedding simple-reachability in the moveit launch-file for your robot.

##### Visualization

If you want to visualize a previously calculated workspace, you can run:

```roslaunch simple-reachability visualize_workspace.launch```

Make sure you provide the name of the file you want to run in the *visualization.yaml*.

Now you can visualize the workspace by adding a *Marker* with the topic ```/visualize_workspace/visualization_marker``` in RVIZ.

In the config file you can modify which part of the workspace you can see. For example you could cut it to only see the *LEFT_HEMISPHERE* for better visualization.

### Contribution

Feel free to improve the code or implement some new features. Feature requests or bugs can be reported as *Issue*.
