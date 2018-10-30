# Thorlabs TDC Controller
A rewrite in C++ of the Thorlabs library which is only available for Windows. Based on [APT Communication Protocol](https://www.thorlabs.com/software/apt/APT_Communications_Protocol_Rev_15.pdf) and the [FTDI D2XX](http://www.ftdichip.com/Support/Documents/ProgramGuides/D2XX_Programmer's_Guide(FT_000071).pdf) library for low-level communication to the FTDI FT232BM USB chip.

This is a catkin workspace ROS node, which allows absolute position control for Thorlabs stages at the moment and the library can easily be completed by using commands of APT Communication Protocol. Tested on Ubuntu 16.04 and ROS Kinetic Kame.

## Devices and Functions
In this version, only some functions are implemented to be used with Thorlabs TDC001 in combination with linear stages (MTS50/M-Z8) and rotational stages (CR1/M-Z7). This should give a good overview to implement more commands from APT Communication Protocol. 

## D2XX Drivers
Install [D2XX Drivers](http://www.ftdichip.com/Drivers/D2XX.htm) for your system first. Include the library for compiling.

Then run ldconfig first to update libraries.

There are also other libraries to communicate with FTDI devices, e.g. libftdi1.

In Linux, the VCP driver and D2XX driver are incompatible with each other.  When a FTDI device is 
plugged  in,  the  VCP  driver  must  be  unloaded  before  a  D2XX  application  can  be  run.  Use  the remove module (rmmod) command to do this:
sudo rmmod ftdi_sio
sudo rmmod usbserial

It is also important to give the user enough privileges to access the USB ports (udev rules, user groups).

## Start node

Set serialnumbers/ types and all devices in launch file.

Type 1: linear stage

Type 2: rotational stage

`roslaunch thorlabs_tdc thorlabs_tdc.launch`

## Services and topics
`rostopic echo /thorlabs_x/motor_info_topic`

Homing is necessary before position can be set. Homing of rotational stage sets current position as zero.

`rosservice call /thorlabs_x/thorlabs_homing_service`

Set position in mm (0-50mm) or degree (0-360deg):

`rosservice call /thorlabs_x/thorlabs_set_pos "position_setpoint: 20.0"`

## Hints
- Replace "xxx" in .launch file by TDC serial number.
- Velocity is faulty. Could be an error from Thorlabs.
- TDC status updates only with 10 Hz.
