# Installation Guide

The purpose of this document is to establish an environment for communicating with Crazyflie quadcopters and reading data through Python scripts, for developing the firmware in CrazyFlie, please refer to: [crazyflie-firmware-LL](https://github.com/SFWen2/crazyflie-firmware-LL)

## Python script and development environment for communicating with CrazyFlie

This repository contains crazyflie-based gimbal commander run on a host PC (developed by [MACLAB](http://www.maclab.seas.ucla.edu/)). 

As a beginner, you may need the following tools installed on your machine:
2. Text editor ([VS code](https://code.visualstudio.com/) or others)
2. [python3](https://www.python.org/downloads/)
1. Clone the code from this repository
3. [CrazyFlie Dongle USB Driver](https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/)
4. Optional: [git](https://github.com/git-guides/install-git) for version control
5. Optional: [cfclient](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/) for testing

#### Crazyflie Python API

Config crazyflie library:
```
cd (current_folder)
pip3 install --user ./
```

install numpy and other libraries.


### Communication with Crazyflie

Examples in `./examples`

Define new communication command package: 

* Make changes in `./cflib/crazyflie/commander.py`.

* Config cflib:
```
cd (current_folder)
pip3 install --user ./
```

### FAQ (python script, sending command... etc)

#### How to modify the command trajectory in Python Script?
TBW

#### How to modify the controller gains in Python Script?
TBW








