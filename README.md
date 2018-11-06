# Allegro Hand

### Automatic install

cd to allegro-hand directory and run
```bash
sudo chmod a+x install_dependencies.bash
sudo ./install_dependencies.bash
```

### Installation dependencies

* [Allegro_Hand_Linux_Project](http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Allegro_Hand_Linux_Project)

* [PCAN-Light for LINUX](https://www.peak-system.com/fileadmin/media/linux/index.htm) 
  Should be compiled with chardev support.

* [PCAN-Basic API Linux](https://www.peak-system.com/PCAN-USB.199.0.html)

### Examples on the hardware

Run the hand compensation

```bash
roslaunch ahand ahand.launch
```
This will open rivz and the hand will be runing the ros_controller, which listens to a torque command topic and 
and sends these commands to the hardware via pcan. The gravity compensation torque is computed for all controllers and is allways added to torque command which is sent to the hand.
 
### Troubleshooting

```shell
1534240126.209767: ERROR: failed to scan directory (errno=2) '/sys/class/pcan'
```

