ArduSub
=======

An ArduPilot-based project for remotely operated underwater vehicles.

*****

## Intro ##

The ArduSub project is a subsea vehicle control system designed primarily for ROVs and AUVs. It is based on the *ArduCopter* vehicle-type of the [ArduPilot](https://github.com/diydrones/ardupilot) project. 

The majority of code changes required for the ArduSub are held in the [/ArduSub/](/ArduSub/) directory of this repository. Changes have also been made to a number of libraries:

- **AP_Baro:** Added support for MS58XX pressure sensors for water depth measurement
- **AP_Motors:** Added AP_Motors6DOF and AP_MotorsBlueROV classes 
- **AP_RCMapper:** Added "forward" and "strafe" input axes

## Setup ##

### Hardware ###

The ArduSub project currently supports the following hardware:

- 3DR PixHawk Autopilot
- Any reversible ESCs with PWM control and 1500 centerpoint (Blue Robotics Basic ESC and BlueESC)

Since this is based on the ArduPilot project and includes the ArduPilot Hardware Abstration Layer (HAL), it should also be able to run on a number of other autopilots including the following:

- PixHawk variants (PXRacer, PX4V4)
- Navio+, Navio2 from Emlid
- PXFmini from Erle Robotics
- BBBmini

The following autopilots *are not supported*:

- APM 1 through APM 2.6+

### Topside Control Software ###

The ArduSub project currently only supports topside control through [QGroundControl](http://www.qgroundcontrol.org/) using compatible joysticks and gamepad controllers. Please be sure to use the most recent daily build for bug fixes and improvements.

### ROV Frame ###

The ArduSub project currently supports two frame types:

- 6-DOF Thruster Configuration (for the [BlueRobotics BlueROV](http://bluerobotics.com/store/rov/bluerov/))
- Vectored Thruster Configuration with side-by-side vertical thrusters

The code must be compiled for the correct frame type, defined in AP_MotorsBlueROV.cpp.

### Compilation and Flashing ###

Please refer to the ArduPilot documentation for basic instructions on building the code.

- [Mac Instructions](http://dev.ardupilot.com/wiki/building-px4-with-make-on-mac/)
- [Linux Instructions](http://dev.ardupilot.com/wiki/building-px4-for-linux-with-make/)
- [Windows Instructions](http://dev.ardupilot.com/wiki/building-px4-with-make/)

To compile the ArduSub branch, first `cd ArduSub` to enter the directory and then use one of the following commands.

| Command | Function |
| --- | --- |
| `make px4-v2-bluerov` | Compile for BlueROV thruster configuration |
| `make px4-v2-vectored` | Compile for vectored thruster configuration |

Add `-upload` to the end of the make target to upload. For example:

``` bash
make px4-v2-vectored-upload
```

### Initial Setup ###

1. Compile and upload firmware to the autopilot
2. Open QGroundControl
3. Calibrate gamepad with throttle and yaw on right stick, pitch and roll on left stick.
	- Set to "full down is zero throttle"
4. Calibrate radio using the joystick with the same inputs.
5. Change parameters:
	- DISARM_DELAY to 0
	- ARMING_CHECK to Disabled
	- BRD_SAFETYENABLE to Disabled
	- AHRS_ORIENTATION to Roll90 (or applicable orientation)
	- ATC_ACCEL_Y_MAX to Disabled
	- EK_ALT_NOISE to 0.1
	- EK2_ALT_NOISE to 0.1
6. Setup Power Tab
	- Analog Voltage and Current
	- 10000 mAh
	- Power Sensor: Other
	- Current pin: Pixhawk
	- Voltage pin: Pixhawk
	- Voltage multiplier: 9.174
	- Amps per volt: 14.70
7. Setup Pressure Sensor
	- Change primary baro to 2ndBaro
8. Flight Modes
	- Set all to stabilize
	- Set last flight mode to altHold (Learning mode)
9. Set Up Camera Tilt (if desired)
	- Use camera tab to connect RC8 to Output Channel 8
	- Connect camera tilt servo or gimbal to Ch. 8

### Operation ###

The gamepad controls the ROV during operation. It has been tested with the Microsoft XBox controller and the Logitech F310. The following joysticks and buttons are used:

- **Left Stick:** Forward and strafe input
- **Right Stick:** Throttle and yaw input
- **Start Button:** Arm vehicle
- **Back Button:** Disarm vehicle
- **Y Button:** Switch to althold (learning) mode
- **B Button:** Switch to stabilize (manual) mode
- **Buttonpad Up/Down Arrows:** Tilt camera (if applicable and set up in QGC)

#### Modes ####

In **Stabilize** (manual) mode, the ROV will attempt to remain level and will be easily controllable. Constant attention is needed to maintain depth.

In **Althold** (learning) mode, the ROV will stabilize and also hold its depth. Throttle input will cause the vehicle to ascend or descend and releasing the throttle will cause it to hold its current depth.

## How To Get Involved ##

The ArduSub project is open source and we encourage participation and code contributions: [guidelines for contributors to the ardupilot codebase](http://dev.ardupilot.com/wiki/guidelines-for-contributors-to-the-apm-codebase).

Desired Enhancements and Bugs can be posted to the [issues list](https://github.com/bluerobotics/ardupilot/issues).

## License ##
[Overview of license](http://dev.ardupilot.com/wiki/license-gplv3)

[Full Text](https://github.com/bluerobotics/ardupilot/blob/master/COPYING.txt)
