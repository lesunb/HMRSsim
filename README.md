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
To install it do

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

The project was tested only with ROS [Foxy](https://docs.ros.org/en/foxy/index.html), but it doesn't mean you cannot try with other versions.


Install the package
-------------------

Currently the easiest way to install the package is to do it locally.
After installing Poetry, in the root folder run the command below to install the `HMRsim_lesunb` inside the environment.
Edit mode means any changes you make to `src/simulator` will be reflected in the package.

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

Run
---

Some examples do not use ROS to run, you can run them with no need to install ROS related dependencies.

Below we describe how to run the examples.

> 💡
> The package exports a utility function to help you parse the config.       
> `$ hmrsimcli configtest -f simulation.json`   
>     


Get inside the environment

```bash
$ poetry shell
```

>
> Check the `examples/` folder for different example simulations
>

The file that build a simulation and runs it is `run.py`
To execute the simulation, go inside the example you want to run and do

```bash
$ python run.py [path/to/config.json]
```

Simulations are defined by config objects. You can pass the config to the Simulator class either by a dict object, or by passing the path to a json file.
```python
simulator = Simulator(config)
```

Run using Docker
----------------

If you just want to run a simulation in the project (e.g. you are not developing HMRSim itself) you may opt to run it using a Docker container.
First you build the image, which does what was described above in a Docker image:

```bash
$ docker build --rm -t hmrsim --build-arg example_folder=[path/to/example/folder] .
```
Explaining somethings in the command:

* With the `--build-arg` argument you specify the path to the example folder you want to execute inside the container.
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

% TODO problem with the poetry.lock file, outdated packages.