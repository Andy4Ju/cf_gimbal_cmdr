# Installation Guide

The purpose of this document is to establish an environment for communicating with Crazyflie quadcopters and reading data through Python scripts, for developing the firmware in CrazyFlie, please refer to: [crazyflie-firmware-LL](https://github.com/SFWen2/crazyflie-firmware-LL)

## Python script and development environment for communicating with CrazyFlie

This repository contains crazyflie-based gimbal commander run on a host PC (developed by [MACLAB](http://www.maclab.seas.ucla.edu/)). 

As a beginner, you may need the following tools installed on your machine:
1. Download the code from this repository
2. [python3](https://www.python.org/downloads/) and text editor (sublime / [VS code](https://code.visualstudio.com/)..)
3. [CrazyFlie Dongle USB Driver](https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/)
4. Optional: [git](https://github.com/git-guides/install-git) for version control
5. Optional: [cfclient](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/) for testing


#### Set up Sublime to use Python3

Personal preference. Different text editor/compiler may be used.

Install Sublime-text: 
search online for key and steps.

Install Sublime-merge:
```
sudo apt-get installapt-transport-https
```
search online for key and steps.

Check Python3 path
```
which python3
```

In Sublime click `Tools -> Build System -> New Build System`, and paste (python3 path should be the same as in previous step)
```
{
 "cmd": ["/usr/bin/python3", "-u", "$file"],
 "file_regex": "^[ ]File \"(...?)\", line ([0-9]*)",
 "selector": "source.python"
}
```

Save file as `Python3.sublime-build`.


#### Crazyflie Python API

Config crazyflie library:
```
cd (current_folder)
pip3 install --user ./
```

install numpy and other libraries.


#### Crazyflie clients

`git clone` from Github.

Install required python packages, referring to `README`.


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








