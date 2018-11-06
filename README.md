# Allegro Hand 
<!--[![Build Status](https://travis-ci.org/gpldecha/allegro-hand.svg?branch=master)](https://travis-ci.org/gpldecha/allegro-hand)-->

Tested with ROS Kinetic

### Install
1. Clone the directory into the src folder of your catkin workspace.
2. cd to the allegro-hand directory and run
```bash
sudo chmod +x install_dependencies.sh
sudo ./install_dependencies.sh
```
3. With this setting you will not be able to use gazebo the simulator, only the hardware. The standard apt-get gazebo uses by default the ode physics engine which is unable to handle the small inertia matrices of the hand whilst running in real time. If you want to use gazebo (running with real time factor) you will need the [dart](https://dartsim.github.io/) simulator which requires compiling gazebo from source, for this follow step 4.
4. The script install_gazebo_source.bash will purge all your current gazebo libraries, clone gazebo from the orfoundation, build and install it with dart.   
```bash
sudo chmod +x install_gazebo_source.sh
sudo ./install_gazebo_source.sh
```
5. You can now build your catkin workspace

### Examples on the hardware

```bash
roslaunch ahand ahand.launch
```
This will open rivz and the hand will be runing the ros_controller, which listens to a torque command topic and 
and sends these commands to the hardware via pcan. The gravity compensation torque is computed for all controllers and is allways added to torque command which is sent to the hand.


### References and drivers
You can look at the bottom of the install_dependencies.bash file and you can see each element which is being installed. 
You can aslo take a look at [Allegro_Hand_Linux_Project](http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Allegro_Hand_Linux_Project) which tells you which drivers to install. All these steps are present in nstall_dependencies.bash.

* [PCAN-Light for LINUX](https://www.peak-system.com/fileadmin/media/linux/index.htm) 
  Should be compiled with chardev support.

* [PCAN-Basic API Linux](https://www.peak-system.com/PCAN-USB.199.0.html)


### Troubleshooting

```shell
1534240126.209767: ERROR: failed to scan directory (errno=2) '/sys/class/pcan'
```

