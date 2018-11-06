#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

ROS_DISTRO=$(ls /opt/ros/) 
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

install_gazebo(){
	read -p "Do you want to install gazebo9, this will purge all other installations  (y/n) "  -n 1 -r
	echo
	if [[ $REPLY =~ ^[Yy]$ ]]; then
		printf "${GREEN}removing gazebo\n${NC}"
		apt-get -q --yes --force-yes remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*' > /dev/null
		apt-get -q --yes --force-yes autoremove  > /dev/null
		sudo find /usr/local -name '*gazebo*' | while read line; do
			echo "removing file '$line'" 
			sudo rm -fr $line
		done
		sudo find /usr/include -name '*gazebo*' | while read line; do
			echo "removing file '$line'" 
			sudo rm -fr $line
		done
		sudo find /usr/lib -name '*gazebo*' | while read line; do
			echo "removing file '$line'" 
			sudo rm -fr $line
		done
		apt-get -q --yes --force-yes update  > /dev/null
		printf "${GREEN}adding osrfoundation to sources gazebo\n${NC}"
		sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
		wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
		apt-get --yes update > /dev/null
		install_library ros-${ROS_DISTRO}-gazebo9-ros-pkgs
		install_library ros-${ROS_DISTRO}-gazebo9-ros-control
	else
		echo "ok"
	fi	
}

install_library libncurses5-dev 
install_library libncurses5
install_library libpopt-dev

install_library linux-headers-$(uname -r)
install_peak_linux_driver
install_pcan_basic

install_gazebo
install_library ros-${ROS_DISTRO}-dartsim
install_library ros-${ROS_DISTRO}-forward-command-controller


