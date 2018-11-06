# Allegro Hand
[![Build Status](https://travis-ci.org/gpldecha/allegro-hand.svg?branch=master)]

### Automatic install

cd to allegro-hand directory and run
```bash
sudo chmod a+x install_dependencies.bash
sudo ./install_dependencies.bash
```
After succesfull installation of the dependencies you can build your catkin workspace

### Manual installation 

You can look at the bottom of the install_dependencies.bash file and you can see each element which is being installed. 
You can aslo take a look at [Allegro_Hand_Linux_Project](http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Allegro_Hand_Linux_Project) which tells you which drivers to install. All these steps are present in nstall_dependencies.bash.

### Drivers

* [PCAN-Light for LINUX](https://www.peak-system.com/fileadmin/media/linux/index.htm) 
  Should be compiled with chardev support.

* [PCAN-Basic API Linux](https://www.peak-system.com/PCAN-USB.199.0.html)

### Examples on the hardware

```bash
roslaunch ahand ahand.launch
```
This will open rivz and the hand will be runing the ros_controller, which listens to a torque command topic and 
and sends these commands to the hardware via pcan. The gravity compensation torque is computed for all controllers and is allways added to torque command which is sent to the hand.


### Troubleshooting

```shell
1534240126.209767: ERROR: failed to scan directory (errno=2) '/sys/class/pcan'
```

