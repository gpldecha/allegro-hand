#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'


while ! echo "$PW" | sudo -S -v > /dev/null 2>&1; do
    read -s -p "Enter your sudo password: " PW
    echo
done

ROS_DISTRO=$(ls /opt/ros/)
INSTALL_GAZEBO_FROM_SOURCE=true
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"


install_library(){
	LIBRARY=$(dpkg -S $1 2> /dev/null )
	if [[ -z $LIBRARY  ]];
	then
		printf "${GREEN}installing $1\n${NC}"
		sudo apt-get -q -y install $1 1> /dev/null
	else
		printf "${GREEN}$1 ... ok\n${NC}"
	fi
}

install_peak_linux_driver(){
	if [[ ! -f /usr/include/libpcan.h ]]; then
		printf "${GREEN}installing peak linux driver\n${NC}"
		cd ${DIR}/peak-linux-driver/
		make NET=NO PCC=NO
		sudo make install
		sudo modprobe pcan 1> /dev/null
		cd ${DIR}
	else
		printf "${GREEN}peak linux driver ... ok\n${NC}"
	fi
}

install_pcan_basic(){
	printf "${GREEN}installing pcan basic api\n${NC}"
	cd ${DIR}/pcan-basic/pcanbasic/
	make
	sudo make install
	cd ${DIR}
}


install_library libncurses5-dev
install_library libncurses5
install_library libpopt-dev

install_library linux-headers-$(uname -r)
install_peak_linux_driver
install_pcan_basic

install_library ros-${ROS_DISTRO}-control-toolbox
install_library ros-${ROS_DISTRO}-controller-manager
install_library ros-${ROS_DISTRO}-transmission-interface
install_library ros-${ROS_DISTRO}-joint-limits-interface
install_library ros-${ROS_DISTRO}-forward-command-controller
