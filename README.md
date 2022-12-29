[![Build Status](https://travis-ci.org/lesunb/HMRSsim.svg?branch=master)](https://travis-ci.org/lesunb/HMRSsim)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/b9b2abf80de34584a596147b099f4473)](https://app.codacy.com/gh/gabrielsr/hmrssim?utm_source=github.com&utm_medium=referral&utm_content=gabrielsr/hmrssim&utm_campaign=Badge_Grade_Settings)
[![codecov](https://codecov.io/gh/lesunb/hmrssim/branch/master/graph/badge.svg)](https://codecov.io/gh/lesunb/hmrssim)

Heterogeneous Multi-Robots Systems Simulator
======================================================


Install for Dev
-------------
Dependencies: python 3.8+, pip, poetry


Install dependencies
--------------------

The poject runs inside a virtual environment managed by [Poetry](https://python-poetry.org/).
To install it do:

```bash
$ pip install poetry
```


Install ROS dependencies (opitional)
------------------------------------

To install ROS dependecies run the commands below:


```bash
$ apt-get update
$ apt-get install -y curl build-essential ros-foxy-rosbridge-server ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-moveit-msgs '~ros-foxy-turtlebot3-.*'
```

The project was tested only with ROS [Foxy](https://docs.ros.org/en/foxy/index.html), but it doesn't mean you cannot try with other versions. Just be aware that you'll have to change some commands.


Install the package
-------------------

Currently the easiest way to install the package is to do it locally.
After installing Poetry, in the root folder run the command below to install the `HMRsim_lesunb` inside the environment.

```bash
$ poetry install
```
Check if the package was installed.

```bash
$ poetry run pip list
# ...
# HMRsim-lesunb            0.0.1
# ...
```

Watch the simulation with Seer via ROS
--------------------------------------

Seer is a component of this project that visually shows the simulation in a web browser.
It comunicates with HMR Sim throught [Rosbridge](http://wiki.ros.org/rosbridge_suite).
To install Rosbridge for ROS Foxy use

```bash
sudo apt-get install ros-foxy-rosbridge-server
```

To run it, open a separated terminal, source ROS and do

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

ROS examples uses Seer throught another Git project, [here](https://github.com/daniloinacio/ROSeer) it is.
Clone the project and run it after ROS Bridge and before HMR Sim (running instructions are in the ROSeer project).

> 💡
> HMR Sim doesn't have a handshake process with ROSeer yet, that's why you have to run it before running HMR Sim for now.
> 

Run
---

Some examples do not use ROS to run, you can run them with no need to install ROS related dependencies.

Below we describe how to run the examples.

> 💡
> The package exports a utility function to help you parse the config.       
> `$ hmrsimcli configtest -f simulation.json`   
>     


First, get inside the environment

```bash
$ poetry shell
```

> 💡
> Check the `examples/` folder for different example simulations
>

Simulations are defined by config objects. You can pass the config to the Simulator class either by a dict object, or by passing the path to a json file. Here's an example with a dict object in python:

```python
simulator = Simulator(config)
```

The file that build a simulation and runs it is called `run.py`.
To execute the simulation, go inside the example you want to run the command below (assuming an example that uses a json config file).

```bash
$ python run.py [path/to/config.json]
```

Run using Docker
----------------

If you just want to run a simulation in the project (e.g. you are not developing HMRSim itself) you may opt to run it using a Docker container.

First you build the image, which does what was described above in a Docker image:

```bash
$ docker build --rm -t hmrsim --build-arg example_folder=[path/to/example/folder] .
```
Explaining somethings in the command:

* With the `--build-arg` argument you specify the path to the example folder you want to execute inside the container (notice that there is a pattern with the structure and name of files in these examples folders).
* If you don't want to set this variable everytime you build, you can set the `example_folder` parameter in the `Dockerfile`.

Then you can run simulations inside the container by using the command below:

```bash
$ docker run hmrsim:latest
```

Dependency
----------

Add New Dependency
------------------

To add new dependencies use the following command.

```console
$ poetry add [name]
$ poetry install
```

This command will add the dependency to the project.


Troubleshooting
---------------

* Your operational system may not have the ROS version used here to run (foxy) the simulation, if that happens try another version.
* If you're having trouble with the version of the `poetry.lock` when building the container, maybe your `poetry.lock` is modified, checkout the changes and try again.
* If you get an error in ROS Seer try stopping HMR Sim (not ROSeer), reloading the page and starting HMR Sim again. This should be solved after an implementation of a handshake process between HMR Sim and ROSeer.