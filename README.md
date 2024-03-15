# Installation Guide
### Setting up environment, designing controller and implement on Crazyflie

This repository contains crazyflie-based gimbal commander run on a host PC (developed by [MACLAB](http://www.maclab.seas.ucla.edu/)). 

### install git

TBW

### install the usb driver for CrazyFlie dongle
https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/


### Communication with Crazyflie

Examples in `./examples`

Define new communication command package: 

* Make changes in `./cflib/crazyflie/commander.py`.

* Config cflib:
```
cd (current_folder)
pip3 install --user ./
```

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


### FAQ (python script, sending command... etc)

#### How to modify the command trajectory in Python Script?
TBW

#### How to modify the controller gains in Python Script?
TBW


## Low-level controller design and uploading

You can choose either Simulink-based method or directly write C code.

### C code template

git clone from my repositoty: https://github.com/SFWen2/crazyflie-firmware-LL.git

![image](https://github.com/SFWen2/cf_gimbal_cmdr/assets/146141804/3d2166d7-f7f5-4a57-80aa-81995d400973)

The Gimbal2D controller template is set to be used for alpha-beta control.

The reference can be found here:

[An Over-Actuated Multi-Rotor Aerial Platform and Iterative Learning Control Applications](https://escholarship.org/uc/item/4pb023fz)



Navigate to `crazyflie-firmware/src/modules/src/controller/controller_Gimbal2D.c`.

find the function: void Gimbal2D_controller(){}, you can modify the control law here.

for example, if you want to do P controller, here's what it looks like:

![image](https://github.com/SFWen2/cf_gimbal_cmdr/assets/146141804/b1ad2a65-a96a-4dd1-a497-b68b7e34104e)


The data structure in controller_Gimbal2D.h:

Gimbal_U has all the input information from communications
Gimbal_Y has all the state information
Gimbal_P has all the parameters

The runtime function in controller_Gimbal2D.c is already set and running in 500Hz, only the controller function needs to be modified.

If you need to add parameters/debug states such that you can monitor them from python script, scroll to the end of `/controller_Gimbal2D.c`. 

You can add some variables with the name you want.





### Design in Simulink

Could use latest version of Matlab and Simulink in Windows. Haven't tried using Matlab in Linux yet.

Crazyflie configuration:

<img src='./image/low_level_config.jpg' alt='quad config' width='400'/>

* Data type throughout Simulink model should be `'single'` or `'inherit'`, unless specified. If designing direct output to each motor, output data type should be `'uint16'` (value range from `0~65535`).

* Specify solver to be fixed-step, and step size 0.002 (500 Hz).

* Use Embedded code generator. Could use C/C++ code advisor to optimize the code, but 'default parameter behavior' under `Configuration Parameters -> Code Generation -> Optimization` should be 'tunable', if any.

* If any tunable parameters, set default values for them in Matlab workspace or Simulink Model workspace.

* Generate code. Should generate a folder `***_ert_rtw`.



### Upload code to Crazyflie

#### Set reference for new controller

* Copy folder `***_ert_rtw` to `crazyflie-firmware/src/modules/src/`.

* If it needs to configure data format of wireless communication, create new receive data packet in `crtp_commander_generic.c`.

* New `controller_***.c`, `controller_***.h`. Define inputs, outputs and parameters. Set `LOG_GROUP` and `PARAM_GROUP`.

* Reference of new controller in `controller.c` (`DEFAULT_CONTROLLER`, `ControllerFcns`) and `controller.h`.

* In `crazyflie-firmware/Makefile`, set:
	* Crazyflie sources (path)
	* Source files config (new `.o` files from `.c` files)
	* Compilation config (path)

#### Flow of data in firmware

* Packet sent from API (`commander.py` in `crazyflie-lib-python/cflib/crazyflie`).

* `crtp_commander_generic.c` in firmware, store in `setpoint`.

* `controller_***.c`

	Map `setpoint/sensor/state` to Simulink input ports;

	Output is either:

	In PID, `roll, pitch, yaw, thrust` that directly control motors by `powerDistribution()` in `power_distribution_stock.c`;

	Or in motor commands, `motorSetRatio()`.


#### Make code and upload

* 
```
cd crazyflie-firmware-LL
make
```

* Connect crazyradio PA; long-press button on QC for 1.5 sec (enter load mode), you will see only the two blue LED blinking.

* 
```
make cload
```

If problem occurs during or after the make and upload, try

```
make clean
```

and proceed again.


### FAQ (controller implementation, build, upload... etc)






