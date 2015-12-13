# nayan_slave_firmware
Implements Loiter Code from ArduCopter on the nayan HLP

## Organization: 
*System files contain important functions for running RTOS and other functionalities and should not be changed

autopilot_math: implements key mathematical structures and operations
config.h: contains all parameters
inertial_nav: implements INS code on the FCU
intercomm: System file. Handles communication between HLP and LLP
mcuconf.h: System file
odroid_comm: handles communication with an odroid system
params: implements saving and loading parameters
position_controller: implements position controller
Setup: System file
stubs: System file
wp_nav: Handles loiter and waypoint navigation

## Flashing on the NAYAN HLP
[nayan_slave_interface](http://aus.co.in/wiki/Nayan_AP_Slave_Processor)
