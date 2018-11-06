#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

ROS_DISTRO=$(ls /opt/ros/) 
INSTALL_GAZEBO_FROM_SOURCE=true
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"


install_library(){
	LIBRARY=$(dpkg -S $1 )
	if [[ -z $LIBRARY  ]]; 
	then
		printf "${GREEN}installing $1\n${NC}"
		apt-get -qq --yes --force-yes install $1
	else
		printf "${GREEN}$1 ... ok\n${NC}"
	fi
}

install_peak_linux_driver(){
	if [[ ! -f /usr/include/libpcan.h ]]; then
		printf "${GREEN}installing peak linux driver\n${NC}"
		cd ${DIR}/peak-linux-driver/
		make NET=NO
		make install
		modprobe pcan
		cd ${DIR}
	else
		printf "${GREEN}peak linux driver ... ok\n${NC}"
	fi
}

install_pcan_basic(){
	if [[ !  /usr/include/PCANBasic.h ]]; then
		printf "${GREEN}installing pcan basic api\n${NC}"
		cd ${DIR}/pcan-basic/pcanbasic/
		make 
		make install
		cd ${DIR}
	else
		printf "${GREEN}pcan basic api ... ok\n${NC}"
	fi
}


install_library libncurses5-dev 
install_library libncurses5
install_library libpopt-dev

install_library linux-headers-$(uname -r)
install_peak_linux_driver
install_pcan_basic

install_library ros-${ROS_DISTRO}-gazebo9-ros-pkgs
install_library ros-${ROS_DISTRO}-gazebo9-dev
install_library ros-${ROS_DISTRO}-gazebo9-ros-control

install_library ros-${ROS_DISTRO}-forward-command-controller


