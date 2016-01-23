## Nayan Slave Firmware

BareBone code template for Nayan Slave Processor written in [chibios](http://www.chibios.org/dokuwiki/doku.php)

### Flashing on the Slave Firmware
[Slave_Processor_Setup](http://aus.co.in/wiki/Slave_Processor_Setup)


###Nayan Wiki
Visit [http://aus.co.in/wiki](http://aus.co.in/wiki)

###Chibios Documentation
Visit [http://chibios.sourceforge.net/html/](http://chibios.sourceforge.net/html/) 

###MAVLink
Visit [http://qgroundcontrol.org/mavlink/start](http://qgroundcontrol.org/mavlink/start)


##Overview

###Sensor Variables
* <code>Sensor_IMU sens_imu</code> holds latest acceleration, angular rates and attitude from IMU
* <code>Sensor_Pose sens_pos</code> holds latest latitude, longitude, velocities in NED from GPS and altitude from barometer. 
* <code>Sensor_ExtPos sens_cv</code> holds vision position estimates from odroid
* <code>uint16_t rc_in[7]</code> holds latest radio control inputs.

###Control Variables
* <code>bool_t dmc</code> control motors direcly if TRUE. If FALSE, then provids setpoints to master controller.
* <code>uint16_t control_command[4]</code> accepts direct motor commands if <code>dmc = TRUE</code> or master controller setpoints if <code>dmc = FASLE</code>


