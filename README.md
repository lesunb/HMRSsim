[![Build Status](https://travis-ci.org/lesunb/HMRSsim.svg?branch=master)](https://travis-ci.org/lesunb/HMRSsim)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/b9b2abf80de34584a596147b099f4473)](https://app.codacy.com/gh/gabrielsr/hmrssim?utm_source=github.com&utm_medium=referral&utm_content=gabrielsr/hmrssim&utm_campaign=Badge_Grade_Settings)
[![codecov](https://codecov.io/gh/lesunb/hmrssim/branch/master/graph/badge.svg)](https://codecov.io/gh/lesunb/hmrssim)

Heterogeneous Multi-Robots Systems Simulator
======================================================


Install for Dev
-------------
Dependencies: python 3.8+, pip

Install pipenv
------------- 

pipenv easy the process of managing python dependencies

Install dependencies
--------------------

Inside the project folder (after clone)

```console
$ pip install pipenv
$ cd HMRSsim
$ pipenv install
$ pipenv shell
(hmrssim env) % pipenv install --dev
```

Install the package
-------------------

Currently the easier way to install the package is to do it locally. In the root folder run the command below to install the `HMRsim_lesunb` package in edit mode. Edit mode means any changes you make to `src/simulator` will be reflected in the package.

```bash
pip install -e .
```
Check the package was installed.   

```bash
pip list
# ...
# HMRsim-lesunb            0.0.1
# ...
```
Run
---

Simulations are defined in by config objects. You can pass the config to the Simulator class either by a dict object, or by passing the path to a json file.  
```python
simulator = Simulator(config)
```

> 💡     
> The package exports a utility function to help you parse the config.       
> `$ hmrsimcli configtest -f simulation.json`   
>     


The file that build a simulation and runs it is `run.py`
To execute the simulation, run
```bash
$ python run.py [path/to/config.json]
```

> 💡     
> Check the `examples/` folder for different example simulations    
>     

Run using Docker
----------------

If you just want to run a simulation in the project (e.g. you are not developing HMRSim itself) you may opt to run it using a Docker container. First you build the image, which does exactly what was described above in a Docker image

```
$ docker build --rm -t hmrsim .
```

Then you can run simulations inside the container by using the command below. 

```
$ docker run -it -v ${PWD}/examples/hospitalSimulation:/usr/app hmrsim:latest python run.py ./simulation.json
```

This command has a few important parts:
* `-v ${PWD}/examples/hospitalSimulation:/usr/app` - Connects the folder `${PWD}/examples/hospitalSimulation` in the host machine to the folder `/usr/app` in the container. The working directory inside the container is `/usr/app`. You should change `examples/hospitalSimulation` by the path to the root folder of your simulation project.

* `hmrsim:latest` - Indicates what image to use

* `python run.py ./simulation.json` - Command to start the simulation, as described above. Notice that `run.py` is actually `/usr/app/run.py` inside the container. Same thing for `simualtion.json`.

Dependency
----------

Add New Dependency
------------------

To add new dependencies use the following command.

```console
$ pipenv install [name]
```

This command will add the dependency to the Pipfile and Pipfile.lock assuring that the execution can be reproduced in another environment (after dependencies are updated with `pipenv install` command )

Add New Dev Dependency
----------------------
Same as previous dependencies, but for development libraries such as the ones used for test.

```console
$ pipenv install [name] --dev
```
Note that other systems after pulling updates will need a reexecution of `pipenv install --dev`
