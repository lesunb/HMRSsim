[![Build Status](https://travis-ci.org/lesunb/HMRSsim.svg?branch=master)](https://travis-ci.org/lesunb/HMRSsim)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/b9b2abf80de34584a596147b099f4473)](https://app.codacy.com/gh/gabrielsr/hmrssim?utm_source=github.com&utm_medium=referral&utm_content=gabrielsr/hmrssim&utm_campaign=Badge_Grade_Settings)
[![codecov](https://codecov.io/gh/lesunb/hmrssim/branch/master/graph/badge.svg)](https://codecov.io/gh/lesunb/hmrssim)

Heterogeneous Multi-Robots Systems Simulator
======================================================

Env Depencies
-------------
python 3, pip

Install pipenv
------------- 

pipenv easy the process of managing python dependencies

PIP
```console
$ pip install pyenv
```

Alternatively, macOS brew
```console
$ brew install pipenv 
```

Install dependencies
--------------------

Inside the project folder (after clone)

```console
$ pyenv install 3.8.0
$ pip install pipenv
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

The file that build a simulation and runs it is `run.py`
To execute the simulation, run
```bash
$ python run.py [path/to/config.json]
```

> 💡     
> Check the `examples/` folder for different example simulations    
>     

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
