# base_serial

A [ROS](http://www.ros.org) node that enables communication with a robot base over a serial link.
On the other end of this link is [cherokey_slave](https://github.com/icboredman/cherokey_slave) processor, implemented using Teensy 3.2 module for the purpose of controlling wheels, collecting odometry and managing battery.

In addition, the node reads CPU temperature and sends it to another device (in this case Arduino Leonardo embedded within LattePanda board) which, in turn, uses it to control cooling fan speed.
The temperature is also published as a ROS message under topic *temperature*.

### Prerequisites

#### Serial Communication Library by William Woodall
* get from here: https://github.com/wjwwood/serial
* before `make`, change install location: `cmake -DCMAKE_INSTALL_PREFIX=/opt/serial`
* enable access to serial device: `sudo gpasswd --add ${USER} dialout`
* add to ~/.bashrc: `source /opt/serial/setup.bash`


---
More info about the project is here: https://boredomprojects.net/index.php/projects/robot-navigation-using-stereo-vision-part-2
