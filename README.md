# Allegro Hand 
<!--[![Build Status](https://travis-ci.org/gpldecha/allegro-hand.svg?branch=master)](https://travis-ci.org/gpldecha/allegro-hand)-->

Tested with ROS Kinetic, Gazebo 9, ODE and DART

### Install allegro-hand hardware
1. Clone the directory into the src folder of your catkin workspace.
2. cd to the allegro-hand directory and run
```bash
sudo chmod +x install_dependencies.sh
sudo ./install_dependencies.sh
```
With this setup you will not be able to use gazebo the simulator. The standard "apt-get install gazebo" uses by default the ode physics engine which is unable to handle the small inertia matrices of the hand whilst maintaining real time performance. 

If you want to use gazebo (running with a real time factor) you will need the [dart](https://dartsim.github.io/) physics engine gazebo plugin which requires compiling gazebo from source. Follow the steps outlined in **Install allegro-hand gazebo**. If you don't want to use the gazebo simulator, you are done, just compile your catkin workspace.

### Install allegro-hand gazebo

1. The script *install_gazebo_source.sh* **will purge all your gazebo libraries**, clone gazebo from the orfoundation, build and install it with dart.   
```bash
sudo chmod +x install_gazebo_source.sh
sudo ./install_gazebo_source.sh
```
2. Delete the CATKIN_IGNORE file from the ahand_gazebo folder
3. You can now build your catkin workspace

### Running on the hardware

```bash
roslaunch ahand hw.launch
```
### Running on the gazebo simulator

```bash
roslaunch ahand sim.launch
```

### Troubleshooting

If you get the following error:
```bash
1555577332.132839: ERROR: failed to scan directory (errno=2) '/sys/class/pcan'
```
it is most likely that the pcan dirver was incorrectly installed. If the /proc/pcan file does 
not exist run the following:

```bash
cd allegro-hand/peak-linux-driver
make clean
make NET=NO PCC=NO
sudo make install
sudo modprobe pcan 1> /dev/null
```
then check if the pcan file exists:
```bash
cat /proc/pcan
```
if it does then this error is likely to be fixed.



