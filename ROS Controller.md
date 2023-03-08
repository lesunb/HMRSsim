Instructions of how to control HMR Sim robots via ROS
=====================================================

You can control the robots in the simulation with ROS via python/C++ or terminal.
Here we you show how to use the terminal.
First, don't forget to source your ROS distribuction.

Making the robot move
---------------------

To control the robot navigation, HMR Sim uses an interface defined by the [Navigation 2](https://navigation.ros.org/) project for that.
[Here](https://github.com/ros-planning/navigation2/tree/main/nav2_msgs) is the messages project in the Navigation 2 project and [here](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action) is the file that defines the structure of a navigation action used to control the robots inside HMR Sim.

To send actions via ROS throught terminal use `ros2 action <something>`.
Here is an example for HMR Sim:

```bash
ros2 action send_goal navigate_to_pose/wall_e nav2_msgs/action/NavigateToPose "{pose: {  pose: { position: {x: 525, y: 205, z: 1} } } }" --feedback
```

Explaining...

* `send_goal` tells us we are sending a new objective.
* `navigate_to_pose/wall_e` is the name of the action used by HRMR Sim to control a robot named **wall_e**.
* `nav2_msgs/action/NavigateToPose` is the type of the action that's being sent.
* `"{pose: {  pose: { position: {x: 525, y: 205, z: 1} } } }"` is the structure of the goal being sent to HMR Sim.
* `--feedback` Makes the command hold the terminal to wait for feedbacks.

In this example we send an objective to a robot called **wall_e** to go to the position `x=525, y=205`.

Making the robot grab and drop
------------------------------

A robot can grab and drop objects if the necessary systems are being used in the simulation.
HMR Sim uses an interface defined by the [MoveIt](https://moveit.picknik.ai/foxy/index.html) project to make a robot grad or drop objects. [Here](https://github.com/ros-planning/moveit_msgs) is the project with the messages used here, [here](https://github.com/ros-planning/moveit_msgs/blob/master/action/Pickup.action) is the file that defines the pickup interface and [here](https://github.com/ros-planning/moveit_msgs/blob/master/action/Place.action) is the file that defines the place interface.

Here is an example of how to send a goal to a robot to grab an object:

```bash
ros2 action send_goal wall_e/grab moveit_msgs/action/Pickup "{ target_name: 'medicine' }" --feedback
```

Explaining...

* `wall_e/grab` is telling that we are sending a **grab** objective to a robot named **wall_e**.
* `moveit_msgs/action/Pickup` is the type of the goal being sent.
* `"{ target_name: 'medicine' }"` is the structure of the message being sent.

In this example we send a goal to **wall_e** to grab an object called **medicine**.
Below, there is a command example to make **wall_e** drop it.

```bash
ros2 action send_goal wall_e/drop moveit_msgs/action/Place "{ attached_object_name: 'medicine' }" --feedback
```

Controlling with python
-----------------------

[Here](https://github.com/yellak/learning-ros/blob/main/hmrsim_tester.py) is an example of controlling a robot in HMR Sim with python, just sending a *go to pose* goal.

[Here](https://github.com/yellak/learning-py-trees/blob/main/go_to_pose_tree.py) is another example of sending a *go to pose* goal but using [py-trees](https://py-trees.readthedocs.io/en/devel/).