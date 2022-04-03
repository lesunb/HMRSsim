HMRSim running with Ros
=============================

In this tutorial, we show you how to run the HMRSim as a ROS Node.

Dependencies
------------

* python 3.8+
* ros (foxy)
* colcon

Preparing the environment
-------------------------

Setup the project as in the main README file of this project.

Source the ROS distribution you're using. You can find a guide to install and configure ROS [here](https://docs.ros.org/en/foxy/Installation.html). This implementation were only tested with *ros foxy*, but you can try another version if you want.

Go to the folder *src/ros2_ws* in this project and run the folowing commands (you may have to exit virtual environment for this to work):

```bash
rosdep install -i --from-path src --rosdistro foxy -y

colcon build --packages-select hmrsim_ros

. install/setup.bash
```


Now you need to configure a *simulation file* where you point to a *draw.io* file with the map and logging file. There is one example in the folder *examples/navigationSimulationRos* with some configurations, but you have to setup *map* and *loggerConfig* to your case.

If not already, go into virtual environment with `pipenv shell`.

Running
-------

The simulation will be expecting commands through ROS, so first we recommend you to create a ROS node separetaly that sends messages to HMRSim through the topic `movebase/robot`, each message received will be logged. In the `MoveBaseSystem` system you will find how HMRSim will interpret the messages and thus how you can send them.

Run the simulation with `ros2 run hmrsim_ros simulation <simulation_file>`, Changing `<simulation_file>` to the path of your simulation file.